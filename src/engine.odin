package pelorus

import "core:fmt"
import "core:math"
import core "core"
import filter "filter"
import io "io"
import mapping "mapping"
import odometry "odometry"
import planning "planning"
import sensor "sensor"
import server "server"
import tracking "tracking"

//
// Top-level Pelorus Perception and Planning Engine.
// Integrates LiDAR-inertial state estimation, continuous-time motion deskewing,
// TSDF mapping, 3D dynamic obstacle tracking, and kinodynamic trajectory planning.
//
PelorusEngine :: struct {
	config:          io.EngineConfig,
	eskf:            odometry.Eskf,
	traj_buffer:     core.TrajectoryBuffer,
	keyframe_mgr:    odometry.KeyframeManager,
	icp_solver:      odometry.IcpSolver,
	voxel_filter:    filter.VoxelGridFilter,
	ground_seg:      filter.GroundSegmenter,
	clusterer:       tracking.EuclideanClusterer,
	map_table:       mapping.VoxelHashTable,
	tsdf_integrator: mapping.TsdfIntegrator,
	esdf:            mapping.EsdfMap,
	mot_tracker:     tracking.MultiObjectTracker,
	planner:         planning.RrtStarPlanner,
	active_traj:     planning.QuinticTrajectory3D,
	telemetry:       server.TelemetryServer,
	target_cloud:    sensor.PointCloud,
	current_pose:    core.Transform3D,
	current_vel:     core.Vec3,
	frame_seq:       u64,
	last_timestamp:  f64,
	has_map_target:  bool,
	is_initialized:  bool,
}

//
// Initialize PelorusEngine with specified configuration parameters.
//
pelorus_engine_init :: proc(engine: ^PelorusEngine, cfg: io.EngineConfig) -> bool {
	engine.config = cfg
	engine.current_pose = core.transform3d_identity()
	engine.current_vel = core.Vec3{0, 0, 0}
	engine.frame_seq = 0
	engine.last_timestamp = 0.0
	engine.has_map_target = false

	// 1. Initialize State Estimator (ESKF)
	initial_state := sensor.NavState{
		timestamp  = 0.0,
		pose       = core.transform3d_identity(),
		velocity   = core.Vec3{0, 0, 0},
		accel_bias = core.Vec3{0, 0, 0},
		gyro_bias  = core.Vec3{0, 0, 0},
		gravity    = core.Vec3{0, 0, -9.81},
	}
	odometry.eskf_init(&engine.eskf, initial_state)

	// 2. Initialize Continuous-time Trajectory Buffer
	core.trajectory_buffer_init(&engine.traj_buffer, 1024)

	// 3. Initialize Keyframe Manager
	odometry.keyframe_manager_init(&engine.keyframe_mgr, 1.0, 15.0, 5.0, 5.0, 3)

	// 4. Initialize ICP Solver
	icp_params := odometry.registration_params_default()
	icp_params.max_iterations = cfg.icp_max_iters
	odometry.icp_solver_init(&engine.icp_solver, icp_params, 2048)

	// 5. Initialize Voxel Grid Filter and Ground Segmenter
	filter.voxel_grid_init(&engine.voxel_filter, core.Vec3{cfg.voxel_size, cfg.voxel_size, cfg.voxel_size}, 4096)
	filter.ground_segmenter_init(&engine.ground_seg, 0.15, 15.0, -0.1)

	// 6. Initialize Euclidean Clusterer
	tracking.clusterer_init(&engine.clusterer, 0.4, 6, 400)

	// 7. Initialize TSDF Mapping
	mapping.voxel_hash_init(&engine.map_table, cfg.voxel_size, 16384)
	tsdf_params := mapping.tsdf_params_default(cfg.voxel_size)
	mapping.tsdf_integrator_init(&engine.tsdf_integrator, &engine.map_table, tsdf_params)

	// 8. Initialize ESDF Clearance Map
	mapping.esdf_map_init(&engine.esdf, cfg.voxel_size * 2.0, 48, cfg.esdf_max_dist)

	// 9. Initialize Target Cloud container
	sensor.point_cloud_init(&engine.target_cloud, 2048)

	// 10. Initialize Multi-Object Tracker
	tracking.tracker_init(&engine.mot_tracker, 2, 3, 3.0)

	// 11. Initialize RRT* Motion Planner and Spline Trajectory
	planning.rrt_star_init(&engine.planner, 0.5, 1.2, 500, 0.3)
	planning.quintic_trajectory_init(&engine.active_traj)

	// 12. Initialize Telemetry Server
	srv_cfg := server.server_config_default()
	server.telemetry_server_init(&engine.telemetry, srv_cfg)

	engine.is_initialized = true
	return true
}

//
// Destroy and release all engine memory and background servers.
//
pelorus_engine_destroy :: proc(engine: ^PelorusEngine) {
	if !engine.is_initialized do return

	server.telemetry_server_destroy(&engine.telemetry)
	planning.quintic_trajectory_destroy(&engine.active_traj)
	planning.rrt_star_destroy(&engine.planner)
	tracking.tracker_destroy(&engine.mot_tracker)
	tracking.clusterer_destroy(&engine.clusterer)
	sensor.point_cloud_destroy(&engine.target_cloud)
	mapping.esdf_map_destroy(&engine.esdf)
	mapping.voxel_hash_destroy(&engine.map_table)
	filter.voxel_grid_destroy(&engine.voxel_filter)
	odometry.icp_solver_destroy(&engine.icp_solver)
	odometry.keyframe_manager_destroy(&engine.keyframe_mgr)
	core.trajectory_buffer_destroy(&engine.traj_buffer)

	engine.is_initialized = false
}

//
// Process high-frequency IMU sensor measurement.
//
pelorus_engine_process_imu :: proc(
	engine: ^PelorusEngine,
	accel: core.Vec3,
	gyro: core.Vec3,
	timestamp: f64,
) {
	dt := f32(0.01)
	if engine.last_timestamp > 0.0 {
		delta := f32(timestamp - engine.last_timestamp)
		if delta > 0.0001 && delta < 0.1 {
			dt = delta
		}
	}

	odometry.eskf_predict(&engine.eskf, accel, gyro, dt)

	engine.current_pose = engine.eskf.state.pose
	engine.current_vel = engine.eskf.state.velocity
	engine.last_timestamp = timestamp

	// Record in trajectory buffer for continuous deskewing
	core.trajectory_buffer_push(&engine.traj_buffer, core.TrajectoryPose{
		timestamp = timestamp,
		pose      = engine.current_pose,
		twist     = core.Twist6{},
	})
}

//
// Ingest raw LiDAR scan, deskew, match odometry, integrate TSDF, track obstacles,
// and broadcast live perception state.
//
pelorus_engine_process_scan :: proc(
	engine: ^PelorusEngine,
	raw_scan: ^sensor.PointCloud,
	deskewed_out: ^sensor.PointCloud,
) -> core.Transform3D {
	engine.frame_seq += 1

	// Ensure reference pose is in trajectory buffer
	core.trajectory_buffer_push(&engine.traj_buffer, core.TrajectoryPose{
		timestamp = raw_scan.header.timestamp,
		pose      = engine.current_pose,
		twist     = core.Twist6{},
	})

	// 1. Deskew raw point cloud using continuous trajectory buffer
	sensor.deskew_point_cloud(deskewed_out, raw_scan, &engine.traj_buffer, raw_scan.header.timestamp)

	// 2. Downsample deskewed cloud for scan matching
	filtered_cloud: sensor.PointCloud
	sensor.point_cloud_init(&filtered_cloud, 1024)
	defer sensor.point_cloud_destroy(&filtered_cloud)

	filter.voxel_grid_filter(&engine.voxel_filter, &filtered_cloud, deskewed_out)

	// 3. Scan matching against local map
	if !engine.has_map_target {
		// Populate initial target cloud
		clear(&engine.target_cloud.points)
		for pt in filtered_cloud.points {
			append(&engine.target_cloud.points, pt)
		}
		engine.has_map_target = true
	} else {
		// Run Point-to-Plane ICP
		res := odometry.icp_align(&engine.icp_solver, &filtered_cloud, &engine.target_cloud)
		if res.converged {
			odometry.eskf_update_lidar_pose(
				&engine.eskf,
				res.transform.translation,
				res.transform.rotation,
				0.01,
				0.001,
			)

			engine.current_pose = engine.eskf.state.pose
			engine.current_vel = engine.eskf.state.velocity

			// Spawn keyframe if vehicle moved sufficiently
			if odometry.keyframe_should_spawn(&engine.keyframe_mgr, engine.current_pose, raw_scan.header.timestamp) {
				odometry.keyframe_spawn(&engine.keyframe_mgr, engine.current_pose, &filtered_cloud, raw_scan.header.timestamp)
				clear(&engine.target_cloud.points)
				for pt in filtered_cloud.points {
					append(&engine.target_cloud.points, pt)
				}
			}
		}
	}

	// 4. Integrate deskewed scan into TSDF Map
	mapping.tsdf_integrate_point_cloud(&engine.tsdf_integrator, deskewed_out, engine.current_pose.translation)

	// 5. Ground segmentation and dynamic obstacle clustering
	ground_cloud: sensor.PointCloud
	obstacle_cloud: sensor.PointCloud
	sensor.point_cloud_init(&ground_cloud, 512)
	sensor.point_cloud_init(&obstacle_cloud, 512)
	defer sensor.point_cloud_destroy(&ground_cloud)
	defer sensor.point_cloud_destroy(&obstacle_cloud)

	filter.ground_segmenter_segment(&engine.ground_seg, &ground_cloud, &obstacle_cloud, &filtered_cloud)

	clusters := make([dynamic]tracking.PointCluster, 0, 16)
	defer {
		for i in 0 ..< len(clusters) do delete(clusters[i].points)
		delete(clusters)
	}

	tracking.clusterer_extract(&engine.clusterer, &clusters, &obstacle_cloud)

	// Fit 3D bounding boxes to clusters
	detections := make([dynamic]tracking.BoundingBox3D, 0, len(clusters))
	defer delete(detections)

	tracking.fit_bounding_boxes(clusters[:], &detections)

	// 6. Update Multi-Object Tracker (MOT)
	dt_scan := f32(0.1)
	tracking.tracker_step(&engine.mot_tracker, detections[:], dt_scan)

	// 7. Telemetry Broadcast
	sample_path := make([]core.Vec3, 0)
	defer delete(sample_path)

	json_snapshot := io.json_serialize_perception_snapshot(
		raw_scan.header.timestamp,
		engine.frame_seq,
		engine.current_pose,
		engine.current_vel,
		engine.mot_tracker.tracks[:],
		sample_path,
	)
	defer delete(json_snapshot)

	server.telemetry_server_update_state(&engine.telemetry, json_snapshot)
	server.telemetry_server_broadcast(&engine.telemetry, json_snapshot)
	server.telemetry_server_poll(&engine.telemetry)

	return engine.current_pose
}

//
// Plan collision-free trajectory to target 3D goal position.
//
pelorus_engine_plan_trajectory :: proc(
	engine: ^PelorusEngine,
	goal_pos: core.Vec3,
) -> bool {
	// Re-propagate ESDF clearance field from TSDF surface
	mapping.esdf_propagate_from_tsdf(&engine.esdf, &engine.map_table)

	start_pos := engine.current_pose.translation
	bounds := core.AABB{
		min_pt = core.Vec3{
			math.min(start_pos.x, goal_pos.x) - 5.0,
			math.min(start_pos.y, goal_pos.y) - 5.0,
			math.min(start_pos.z, goal_pos.z) - 1.0,
		},
		max_pt = core.Vec3{
			math.max(start_pos.x, goal_pos.x) + 5.0,
			math.max(start_pos.y, goal_pos.y) + 5.0,
			math.max(start_pos.z, goal_pos.z) + 3.0,
		},
	}

	raw_waypoints := make([dynamic]core.Vec3)
	defer delete(raw_waypoints)

	path_ok := planning.rrt_star_plan(&engine.planner, &engine.esdf, start_pos, goal_pos, bounds, &raw_waypoints)
	if !path_ok do return false

	// Generate minimum-jerk spline trajectory
	v0 := engine.current_vel
	a0 := core.Vec3{0, 0, 0}
	vf := core.Vec3{0, 0, 0}
	af := core.Vec3{0, 0, 0}

	traj_ok := planning.generate_minimum_jerk_trajectory(
		raw_waypoints[:],
		v0,
		a0,
		vf,
		af,
		engine.config.max_velocity * 0.7,
		&engine.active_traj,
	)

	return traj_ok
}
