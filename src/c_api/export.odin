package pelorus_c_api

import "base:runtime"
import "core:math"
import core "../core"
import io "../io"
import planning "../planning"
import sensor "../sensor"
import pelorus "../"

//
// C ABI Export Layer for libpelorus.
// Enables zero-overhead dynamic linking from C, C++, Python, Rust, and Julia.
//

@(export)
pelorus_get_version :: proc "c" () -> cstring {
	return "0.8.0"
}

@(export)
pelorus_engine_create :: proc "c" (voxel_size: f32) -> rawptr {
	context = runtime.default_context()

	cfg := io.engine_config_default()
	if voxel_size > 0.01 {
		cfg.voxel_size = voxel_size
	}

	eng := new(pelorus.PelorusEngine)
	if !pelorus.pelorus_engine_init(eng, cfg) {
		free(eng)
		return nil
	}

	return rawptr(eng)
}

@(export)
pelorus_engine_destroy :: proc "c" (handle: rawptr) {
	if handle == nil do return
	context = runtime.default_context()

	eng := cast(^pelorus.PelorusEngine)handle
	pelorus.pelorus_engine_destroy(eng)
	free(eng)
}

@(export)
pelorus_engine_push_imu :: proc "c" (
	handle: rawptr,
	ax, ay, az: f32,
	gx, gy, gz: f32,
	timestamp: f64,
) {
	if handle == nil do return
	context = runtime.default_context()

	eng := cast(^pelorus.PelorusEngine)handle
	accel := core.Vec3{ax, ay, az}
	gyro := core.Vec3{gx, gy, gz}
	pelorus.pelorus_engine_process_imu(eng, accel, gyro, timestamp)
}

@(export)
pelorus_engine_process_points :: proc "c" (
	handle: rawptr,
	points_xyz: [^]f32,
	num_points: i32,
	timestamp: f64,
	out_pose_7d: [^]f32, // [x, y, z, qx, qy, qz, qw]
) -> i32 {
	if handle == nil || points_xyz == nil || num_points <= 0 do return -1
	context = runtime.default_context()

	eng := cast(^pelorus.PelorusEngine)handle

	cloud_in: sensor.PointCloud
	sensor.point_cloud_init(&cloud_in, int(num_points))
	defer sensor.point_cloud_destroy(&cloud_in)

	cloud_in.header.timestamp = timestamp

	for i in 0 ..< num_points {
		idx := int(i * 3)
		x := points_xyz[idx]
		y := points_xyz[idx + 1]
		z := points_xyz[idx + 2]
		sensor.point_cloud_push(&cloud_in, core.Vec3{x, y, z}, 100.0, timestamp, 0)
	}

	deskewed: sensor.PointCloud
	sensor.point_cloud_init(&deskewed, int(num_points))
	defer sensor.point_cloud_destroy(&deskewed)

	pose := pelorus.pelorus_engine_process_scan(eng, &cloud_in, &deskewed)

	if out_pose_7d != nil {
		out_pose_7d[0] = pose.translation.x
		out_pose_7d[1] = pose.translation.y
		out_pose_7d[2] = pose.translation.z
		out_pose_7d[3] = pose.rotation.x
		out_pose_7d[4] = pose.rotation.y
		out_pose_7d[5] = pose.rotation.z
		out_pose_7d[6] = pose.rotation.w
	}

	return 0
}

@(export)
pelorus_engine_plan_trajectory :: proc "c" (
	handle: rawptr,
	goal_x, goal_y, goal_z: f32,
	out_waypoints_xyz: [^]f32,
	max_waypoints: i32,
) -> i32 {
	if handle == nil || max_waypoints <= 0 do return -1
	context = runtime.default_context()

	eng := cast(^pelorus.PelorusEngine)handle
	goal := core.Vec3{goal_x, goal_y, goal_z}

	ok := pelorus.pelorus_engine_plan_trajectory(eng, goal)
	if !ok do return 0

	n_segs := len(eng.active_traj.segments)
	if n_segs == 0 do return 0

	// Sample 20 points along the trajectory
	samples := int(max_waypoints)
	if samples > 20 do samples = 20
	dt := eng.active_traj.total_time / f64(samples - 1)

	for i in 0 ..< samples {
		t := f64(i) * dt
		pos, _, _, _ := planning.quintic_trajectory_sample(&eng.active_traj, t)
		if out_waypoints_xyz != nil {
			out_waypoints_xyz[i * 3 + 0] = pos.x
			out_waypoints_xyz[i * 3 + 1] = pos.y
			out_waypoints_xyz[i * 3 + 2] = pos.z
		}
	}

	return i32(samples)
}

@(export)
pelorus_engine_get_obstacle_count :: proc "c" (handle: rawptr) -> i32 {
	if handle == nil do return 0
	eng := cast(^pelorus.PelorusEngine)handle
	return i32(len(eng.mot_tracker.tracks))
}
