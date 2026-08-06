package pelorus_tests

import "core:fmt"
import "core:math"
import "core:testing"
import core "../src/core"
import filter "../src/filter"
import io "../src/io"
import mapping "../src/mapping"
import odometry "../src/odometry"
import planning "../src/planning"
import sensor "../src/sensor"
import tracking "../src/tracking"

//
// End-to-end Integration Test: Multi-Sensor Ingestion -> ESKF -> TSDF -> Clustering -> MOT -> RRT* Plan.
//
@(test)
test_end_to_end_perception_and_planning_pipeline :: proc(t: ^testing.T) {
	// 1. Setup ESKF State Estimator
	initial_state := sensor.NavState{
		timestamp  = 0.0,
		pose       = core.transform3d_identity(),
		velocity   = core.Vec3{1.0, 0.0, 0.0},
		accel_bias = core.Vec3{0, 0, 0},
		gyro_bias  = core.Vec3{0, 0, 0},
		gravity    = core.Vec3{0, 0, -9.81},
	}

	filter_eskf: odometry.Eskf
	odometry.eskf_init(&filter_eskf, initial_state)

	// Predict IMU state forward 10 steps (0.1 sec total)
	dt := f32(0.01)
	accel := core.Vec3{0, 0, 9.81} // Cancels gravity
	gyro := core.Vec3{0, 0, 0}
	for _ in 0 ..< 10 {
		odometry.eskf_predict(&filter_eskf, accel, gyro, dt)
	}

	testing.expect(t, filter_eskf.state.pose.translation.x > 0.05, "ESKF state should propagate forward along X")

	// 2. Setup Voxel Hash TSDF Mapping
	voxel_size := f32(0.1)
	table: mapping.VoxelHashTable
	mapping.voxel_hash_init(&table, voxel_size, 2048)
	defer mapping.voxel_hash_destroy(&table)

	tsdf_params := mapping.tsdf_params_default(voxel_size)
	integrator: mapping.TsdfIntegrator
	mapping.tsdf_integrator_init(&integrator, &table, tsdf_params)

	// Generate synthetic planar wall at x = 3.0
	cloud_in: sensor.PointCloud
	sensor.point_cloud_init(&cloud_in, 100)
	defer sensor.point_cloud_destroy(&cloud_in)

	for y in -2 ..= 2 {
		for z in 0 ..= 3 {
			sensor.point_cloud_push(
				&cloud_in,
				core.Vec3{3.0, f32(y) * 0.2, f32(z) * 0.2},
				100.0,
				0.1,
				0,
				core.Vec3{-1, 0, 0},
			)
		}
	}

	mapping.tsdf_integrate_point_cloud(&integrator, &cloud_in, core.Vec3{0, 0, 0})
	testing.expect(t, table.active_blocks > 0, "TSDF mapping should occupy voxel blocks")

	// 3. Ground plane segmentation
	ground_cloud: sensor.PointCloud
	obstacle_cloud: sensor.PointCloud
	sensor.point_cloud_init(&ground_cloud, 50)
	sensor.point_cloud_init(&obstacle_cloud, 50)
	defer sensor.point_cloud_destroy(&ground_cloud)
	defer sensor.point_cloud_destroy(&obstacle_cloud)

	seg: filter.GroundSegmenter
	filter.ground_segmenter_init(&seg, 0.15, 15.0, 0.2)
	filter.ground_segmenter_segment(&seg, &ground_cloud, &obstacle_cloud, &cloud_in)

	// Wall points above ground threshold should be routed to obstacle cloud
	testing.expect(t, len(obstacle_cloud.points) > 0, "Obstacle points should be extracted")

	// 4. Euclidean clustering on obstacle cloud
	clusterer: tracking.EuclideanClusterer
	tracking.clusterer_init(&clusterer, 0.4, 5, 200)
	defer tracking.clusterer_destroy(&clusterer)

	clusters := make([dynamic]tracking.PointCluster, 0, 8)
	defer {
		for i in 0 ..< len(clusters) do delete(clusters[i].points)
		delete(clusters)
	}

	cluster_count := tracking.clusterer_extract(&clusterer, &clusters, &obstacle_cloud)
	testing.expect(t, cluster_count >= 1, "Should identify at least 1 cluster from obstacle wall")

	// 5. Fit 3D Oriented Bounding Box
	detections := make([dynamic]tracking.BoundingBox3D, 0, cluster_count)
	defer delete(detections)
	tracking.fit_bounding_boxes(clusters[:], &detections)

	testing.expect(t, len(detections) >= 1, "Should generate at least 1 3D bounding box detection")
	bbox := detections[0]
	testing.expect(t, math.abs(bbox.center.x - 3.0) < 0.3, "Bounding box center X should match wall at x=3.0")

	// 6. Multi-Target Obstacle Tracker update
	mot: tracking.MultiObjectTracker
	tracking.tracker_init(&mot, 2, 2, 3.0)
	defer tracking.tracker_destroy(&mot)

	tracking.tracker_step(&mot, detections[:], 0.1)
	testing.expect_value(t, len(mot.tracks), 1)

	// 7. ESDF Propagation from TSDF Map
	esdf: mapping.EsdfMap
	mapping.esdf_map_init(&esdf, 0.2, 32, 4.0)
	defer mapping.esdf_map_destroy(&esdf)

	mapping.esdf_propagate_from_tsdf(&esdf, &table)

	// Query obstacle clearance at origin (should be clear, dist > 1.0m)
	d_origin, _, ok_origin := mapping.esdf_query_distance_and_gradient(&esdf, core.Vec3{0, 0, 0})
	testing.expect(t, ok_origin, "ESDF origin query should succeed")
	testing.expect(t, d_origin > 1.0, "Origin clearance should be greater than 1 meter")

	// 8. Kinodynamic RRT* Path Planning around obstacle
	planner: planning.RrtStarPlanner
	planning.rrt_star_init(&planner, 0.4, 1.0, 500, 0.2)
	defer planning.rrt_star_destroy(&planner)

	start_pt := core.Vec3{0.0, 0.0, 0.5}
	goal_pt := core.Vec3{1.5, 2.0, 0.5} // Clear goal away from x=3 wall
	bounds := core.AABB{
		min_pt = core.Vec3{-1.0, -1.0, 0.0},
		max_pt = core.Vec3{3.0, 3.0, 2.0},
	}

	planned_path := make([dynamic]core.Vec3)
	defer delete(planned_path)

	path_ok := planning.rrt_star_plan(&planner, &esdf, start_pt, goal_pt, bounds, &planned_path)
	testing.expect(t, path_ok, "RRT* planner should find collision-free path")
	testing.expect(t, len(planned_path) >= 2, "Path must contain at least start and goal waypoints")

	// 9. Quintic Spline Minimum-Jerk Trajectory Generation
	traj: planning.QuinticTrajectory3D
	planning.quintic_trajectory_init(&traj)
	defer planning.quintic_trajectory_destroy(&traj)

	v0 := core.Vec3{1.0, 0.0, 0.0}
	vf := core.Vec3{0.0, 0.0, 0.0}
	traj_ok := planning.generate_minimum_jerk_trajectory(
		planned_path[:],
		v0,
		core.Vec3{0, 0, 0},
		vf,
		core.Vec3{0, 0, 0},
		2.0,
		&traj,
	)
	testing.expect(t, traj_ok, "Trajectory optimization should generate continuous polynomial spline")
	testing.expect(t, traj.total_time > 0.5, "Trajectory duration must be positive")

	// Sample trajectory midpoint
	mid_pos, mid_vel, mid_acc, _ := planning.quintic_trajectory_sample(&traj, traj.total_time * 0.5)
	testing.expect(t, core.vec3_length(mid_vel) > 0.0, "Vehicle should have non-zero velocity at trajectory midpoint")
	testing.expect(t, !math.is_nan(mid_pos.x) && !math.is_nan(mid_pos.y), "Trajectory samples must be numerical finite")
}
