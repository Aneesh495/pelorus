package pelorus_tests

import "core:math"
import "core:testing"
import core "../src/core"
import odometry "../src/odometry"
import sensor "../src/sensor"

@(test)
test_icp_alignment :: proc(t: ^testing.T) {
	params := odometry.registration_params_default()
	params.max_iterations = 40
	params.max_correspondence_dist = 2.0

	solver: odometry.IcpSolver
	odometry.icp_solver_init(&solver, params, 200)
	defer odometry.icp_solver_destroy(&solver)

	// Target cloud: grid of 3D points on ground and two orthogonal vertical walls
	target_cloud: sensor.PointCloud
	sensor.point_cloud_init(&target_cloud, 200)
	defer sensor.point_cloud_destroy(&target_cloud)

	for x in -3 ..= 3 {
		for y in -3 ..= 3 {
			// Horizontal floor (z = 0, normal = [0, 0, 1])
			sensor.point_cloud_push(&target_cloud, core.Vec3{f32(x) * 0.3, f32(y) * 0.3, 0.0}, 100.0, 0.0, 0, core.Vec3{0, 0, 1})
			// Vertical wall X (x = 1.0, normal = [-1, 0, 0])
			sensor.point_cloud_push(&target_cloud, core.Vec3{1.0, f32(y) * 0.3, f32(x) * 0.2 + 0.6}, 100.0, 0.0, 1, core.Vec3{-1, 0, 0})
			// Vertical wall Y (y = 1.0, normal = [0, -1, 0])
			sensor.point_cloud_push(&target_cloud, core.Vec3{f32(x) * 0.3, 1.0, f32(y) * 0.2 + 0.6}, 100.0, 0.0, 2, core.Vec3{0, -1, 0})
		}
	}

	// Source cloud: transformed by known delta translation (0.2, -0.1, 0.05)
	delta_true := core.Transform3D{
		translation = core.Vec3{0.2, -0.1, 0.05},
		rotation    = core.quat_identity(),
	}

	source_cloud: sensor.PointCloud
	sensor.point_cloud_init(&source_cloud, 100)
	defer sensor.point_cloud_destroy(&source_cloud)

	inv_delta := core.transform3d_inverse(delta_true)
	for pt in target_cloud.points {
		p_src := core.transform3d_apply_point(inv_delta, pt.position)
		n_src := core.transform3d_apply_vector(inv_delta, pt.normal)
		sensor.point_cloud_push(&source_cloud, p_src, pt.intensity, pt.timestamp, pt.ring, n_src)
	}

	res := odometry.icp_align(&solver, &source_cloud, &target_cloud)

	testing.expect(t, res.converged, "ICP must converge on structured point clouds")
	testing.expect(t, res.inlier_count > 30, "ICP inlier count insufficient")

	// Solved transform should match delta_true
	trans_err := core.vec3_distance(res.transform.translation, delta_true.translation)
	testing.expect(t, trans_err < 0.05, "ICP estimated translation error too large")
}

@(test)
test_ndt_alignment :: proc(t: ^testing.T) {
	params := odometry.registration_params_default()
	params.max_iterations = 30

	solver: odometry.NdtSolver
	odometry.ndt_solver_init(&solver, params, 0.8, 200)
	defer odometry.ndt_solver_destroy(&solver)

	target_cloud: sensor.PointCloud
	sensor.point_cloud_init(&target_cloud, 100)
	defer sensor.point_cloud_destroy(&target_cloud)

	// Add 10 points in voxel [0, 1]^3
	for i in 0 ..< 10 {
		fi := f32(i) * 0.08
		sensor.point_cloud_push(&target_cloud, core.Vec3{0.1 + fi, 0.2 + fi, 0.3 + fi})
	}
	// Add 10 points in voxel [2, 3]^3
	for i in 0 ..< 10 {
		fi := f32(i) * 0.08
		sensor.point_cloud_push(&target_cloud, core.Vec3{2.1 + fi, 2.2 + fi, 0.3 + fi})
	}

	odometry.ndt_set_target(&solver, &target_cloud)

	// Source cloud shifted by 0.05m
	source_cloud: sensor.PointCloud
	sensor.point_cloud_init(&source_cloud, 100)
	defer sensor.point_cloud_destroy(&source_cloud)

	shift := core.Vec3{0.05, 0.05, 0.0}
	for pt in target_cloud.points {
		sensor.point_cloud_push(&source_cloud, pt.position - shift)
	}

	res := odometry.ndt_align(&solver, &source_cloud)
	testing.expect(t, res.inlier_count > 5, "NDT should find inlier density cells")
}

@(test)
test_eskf_filter :: proc(t: ^testing.T) {
	initial_state := sensor.NavState{
		timestamp  = 0.0,
		pose       = core.transform3d_identity(),
		velocity   = core.Vec3{0, 0, 0},
		accel_bias = core.Vec3{0, 0, 0},
		gyro_bias  = core.Vec3{0, 0, 0},
		gravity    = core.Vec3{0, 0, -9.81},
	}

	filter: odometry.Eskf
	odometry.eskf_init(&filter, initial_state)

	// Stationary IMU measurement canceling gravity: accel = (0, 0, +9.81)
	accel := core.Vec3{0, 0, 9.81}
	gyro := core.Vec3{0, 0, 0}
	dt := f32(0.01)

	// Predict 50 steps
	for _ in 0 ..< 50 {
		odometry.eskf_predict(&filter, accel, gyro, dt)
	}

	// Vehicle should remain stationary
	testing.expect(t, core.vec3_length(filter.state.velocity) < 1e-4, "Stationary velocity drift")
	testing.expect(t, core.vec3_length(filter.state.pose.translation) < 1e-4, "Stationary position drift")

	// Inject LiDAR measurement update at (1.0, 0.5, 0.0)
	meas_pos := core.Vec3{1.0, 0.5, 0.0}
	meas_rot := core.quat_identity()
	odometry.eskf_update_lidar_pose(&filter, meas_pos, meas_rot, 0.01, 0.001)

	// State translation should be corrected toward measurement
	testing.expect(t, filter.state.pose.translation.x > 0.5, "ESKF state position not updated by measurement")
}

@(test)
test_keyframe_manager :: proc(t: ^testing.T) {
	mgr: odometry.KeyframeManager
	odometry.keyframe_manager_init(&mgr, 1.0, 15.0, 5.0, 5.0, 3)
	defer odometry.keyframe_manager_destroy(&mgr)

	cloud: sensor.PointCloud
	sensor.point_cloud_init(&cloud, 10)
	defer sensor.point_cloud_destroy(&cloud)
	sensor.point_cloud_push(&cloud, core.Vec3{1, 2, 3})

	// Frame 0: always spawns first keyframe
	p0 := core.Transform3D{translation = core.Vec3{0, 0, 0}, rotation = core.quat_identity()}
	testing.expect(t, odometry.keyframe_should_spawn(&mgr, p0, 0.0), "First keyframe must spawn")
	id0 := odometry.keyframe_spawn(&mgr, p0, &cloud, 0.0)
	testing.expect_value(t, id0, 0)

	// Frame 1: moved only 0.2m -> should NOT spawn
	p1 := core.Transform3D{translation = core.Vec3{0.2, 0, 0}, rotation = core.quat_identity()}
	testing.expect(t, !odometry.keyframe_should_spawn(&mgr, p1, 0.1), "Small displacement must not spawn keyframe")

	// Frame 2: moved 1.5m -> should spawn
	p2 := core.Transform3D{translation = core.Vec3{1.5, 0, 0}, rotation = core.quat_identity()}
	testing.expect(t, odometry.keyframe_should_spawn(&mgr, p2, 0.5), "Displacement >= 1.0m must spawn keyframe")
	id1 := odometry.keyframe_spawn(&mgr, p2, &cloud, 0.5)
	testing.expect_value(t, id1, 1)

	testing.expect_value(t, len(mgr.keyframes), 2)
}
