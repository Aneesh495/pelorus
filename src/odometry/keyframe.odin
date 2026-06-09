package pelorus_odometry

import "core:math"
import core "../core"
import sensor "../sensor"

//
// Keyframe vertex in spatial pose graph.
// Stores downsampled point cloud and world pose for mapping and loop closure.
//
Keyframe :: struct {
	id:                   u64,
	timestamp:            f64,
	pose:                 core.Transform3D,
	cloud:                sensor.PointCloud,
	accumulated_distance: f32,
}

//
// Loop closure candidate pairing.
//
LoopCandidate :: struct {
	curr_id:       u64,
	prev_id:       u64,
	spatial_dist:  f32,
	temporal_dist: f64,
}

//
// Keyframe lifecycle manager implementing spatial-temporal selection heuristics.
//
KeyframeManager :: struct {
	keyframes:            [dynamic]Keyframe,
	min_trans_dist:       f32, // Minimum translation displacement threshold (meters)
	min_rot_angle_rad:    f32, // Minimum rotation angle threshold (radians)
	max_time_interval:    f64, // Maximum time duration before forcing keyframe (seconds)
	loop_search_radius:   f32, // Spatial radius to search for loop closure candidates
	loop_min_index_diff:  int, // Minimum keyframe count gap to avoid trivial temporal neighbors
	total_distance:       f32,
	owns_memory:          bool,
}

//
// Initialize KeyframeManager with operational thresholds.
//
keyframe_manager_init :: proc(
	mgr: ^KeyframeManager,
	min_trans_m: f32 = 1.0,
	min_rot_deg: f32 = 15.0,
	max_time_s: f64 = 4.0,
	loop_radius_m: f32 = 15.0,
	loop_index_diff: int = 20,
) {
	mgr.keyframes = make([dynamic]Keyframe, 0, 1024)
	mgr.min_trans_dist = min_trans_m
	mgr.min_rot_angle_rad = min_rot_deg * f32(math.PI / 180.0)
	mgr.max_time_interval = max_time_s
	mgr.loop_search_radius = loop_radius_m
	mgr.loop_min_index_diff = loop_index_diff
	mgr.total_distance = 0.0
	mgr.owns_memory = true
}

//
// Destroy KeyframeManager and all owned keyframe clouds.
//
keyframe_manager_destroy :: proc(mgr: ^KeyframeManager) {
	if mgr.owns_memory {
		for i in 0 ..< len(mgr.keyframes) {
			sensor.point_cloud_destroy(&mgr.keyframes[i].cloud)
		}
		delete(mgr.keyframes)
	}
	mgr.keyframes = nil
}

//
// Evaluate whether current pose and timestamp warrant spawning a new keyframe.
//
keyframe_should_spawn :: proc(
	mgr: ^KeyframeManager,
	current_pose: core.Transform3D,
	timestamp: f64,
) -> bool {
	count := len(mgr.keyframes)
	if count == 0 {
		return true // First frame is always a keyframe
	}

	last_kf := mgr.keyframes[count - 1]

	// 1. Spatial translation displacement
	dist := core.vec3_distance(current_pose.translation, last_kf.pose.translation)
	if dist >= mgr.min_trans_dist {
		return true
	}

	// 2. Angular rotation displacement
	inv_rot := core.quat_inverse(last_kf.pose.rotation)
	delta_q := core.quat_mul(inv_rot, current_pose.rotation)
	rot_axis_angle := core.so3_log_quat(delta_q)
	angle := core.vec3_length(rot_axis_angle)
	if angle >= mgr.min_rot_angle_rad {
		return true
	}

	// 3. Temporal elapsed duration
	dt := timestamp - last_kf.timestamp
	if dt >= mgr.max_time_interval {
		return true
	}

	return false
}

//
// Spawn and append a new keyframe to the pose graph.
//
keyframe_spawn :: proc(
	mgr: ^KeyframeManager,
	pose: core.Transform3D,
	cloud: ^sensor.PointCloud,
	timestamp: f64,
) -> u64 {
	count := len(mgr.keyframes)
	new_id := u64(count)

	step_dist: f32 = 0.0
	if count > 0 {
		step_dist = core.vec3_distance(pose.translation, mgr.keyframes[count - 1].pose.translation)
	}
	mgr.total_distance += step_dist

	// Deep copy point cloud into keyframe
	kf_cloud: sensor.PointCloud
	sensor.point_cloud_init(&kf_cloud, len(cloud.points))
	for pt in cloud.points {
		sensor.point_cloud_push(&kf_cloud, pt.position, pt.intensity, pt.timestamp, pt.ring, pt.normal)
	}

	kf := Keyframe{
		id                   = new_id,
		timestamp            = timestamp,
		pose                 = pose,
		cloud                = kf_cloud,
		accumulated_distance = mgr.total_distance,
	}

	append(&mgr.keyframes, kf)
	return new_id
}

//
// Search for potential loop closure candidates near the most recent keyframe.
// Returns list of candidate matches satisfying spatial proximity and temporal separation.
//
keyframe_find_loop_candidates :: proc(
	mgr: ^KeyframeManager,
	candidates_out: ^[dynamic]LoopCandidate,
) -> int {
	count := len(mgr.keyframes)
	if count <= mgr.loop_min_index_diff {
		return 0
	}

	curr_kf := mgr.keyframes[count - 1]
	curr_pos := curr_kf.pose.translation
	search_rad_sq := mgr.loop_search_radius * mgr.loop_search_radius
	found := 0

	// Search historical keyframes excluding recent temporal neighbors
	search_end := count - 1 - mgr.loop_min_index_diff
	for i in 0 ..< search_end {
		hist_kf := mgr.keyframes[i]
		d_sq := core.vec3_distance_sq(curr_pos, hist_kf.pose.translation)

		if d_sq <= search_rad_sq {
			cand := LoopCandidate{
				curr_id       = curr_kf.id,
				prev_id       = hist_kf.id,
				spatial_dist  = math.sqrt(d_sq),
				temporal_dist = curr_kf.timestamp - hist_kf.timestamp,
			}
			append(candidates_out, cand)
			found += 1
		}
	}

	return found
}
