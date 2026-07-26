package pelorus_io

import "core:fmt"
import "core:math"
import "core:strconv"
import "core:strings"
import core "../core"
import odometry "../odometry"
import tracking "../tracking"
import planning "../planning"

//
// Engine configuration parameters loaded or dumped as JSON.
//
EngineConfig :: struct {
	voxel_size:       f32,
	icp_max_iters:    int,
	icp_tolerance:    f32,
	esdf_max_dist:    f32,
	max_velocity:     f32,
	max_acceleration: f32,
	tracking_gating:  f32,
}

//
// Default perception engine parameters.
//
engine_config_default :: proc "contextless" () -> EngineConfig {
	return EngineConfig{
		voxel_size       = 0.1,
		icp_max_iters    = 30,
		icp_tolerance    = 1e-4,
		esdf_max_dist    = 4.0,
		max_velocity     = 4.0,
		max_acceleration = 2.5,
		tracking_gating  = 9.21,
	}
}

//
// Serialize full perception snapshot to JSON string.
//
json_serialize_perception_snapshot :: proc(
	timestamp: f64,
	sequence_id: u64,
	pose: core.Transform3D,
	velocity: core.Vec3,
	tracks: []tracking.Track,
	trajectory_waypoints: []core.Vec3,
) -> string {
	b := strings.builder_make()

	strings.write_string(&b, "{\n")
	fmt.sbprintf(&b, "  \"timestamp\": %.6f,\n", timestamp)
	fmt.sbprintf(&b, "  \"sequence_id\": %d,\n", sequence_id)

	// State pose
	strings.write_string(&b, "  \"pose\": {\n")
	fmt.sbprintf(&b, "    \"position\": [%.4f, %.4f, %.4f],\n", pose.translation.x, pose.translation.y, pose.translation.z)
	fmt.sbprintf(&b, "    \"rotation_quat\": [%.5f, %.5f, %.5f, %.5f],\n", pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)
	fmt.sbprintf(&b, "    \"velocity\": [%.4f, %.4f, %.4f]\n", velocity.x, velocity.y, velocity.z)
	strings.write_string(&b, "  },\n")

	// Tracked obstacles
	fmt.sbprintf(&b, "  \"tracked_obstacles\": [\n")
	for i in 0 ..< len(tracks) {
		trk := tracks[i]
		st := trk.filter.state
		px := st[0]
		py := st[1]
		pz := st[2]
		speed := st[3]
		yaw := st[4]
		vz := st[6]
		extents := trk.filter.dimensions

		strings.write_string(&b, "    {\n")
		fmt.sbprintf(&b, "      \"id\": %d,\n", trk.track_id)
		fmt.sbprintf(&b, "      \"center\": [%.3f, %.3f, %.3f],\n", px, py, pz)
		fmt.sbprintf(&b, "      \"extents\": [%.3f, %.3f, %.3f],\n", extents.x, extents.y, extents.z)
		fmt.sbprintf(&b, "      \"yaw\": %.4f,\n", yaw)
		fmt.sbprintf(&b, "      \"velocity\": [%.3f, %.3f, %.3f],\n", speed * math.cos(yaw), speed * math.sin(yaw), vz)
		fmt.sbprintf(&b, "      \"hits\": %d\n", trk.total_hits)

		if i == len(tracks) - 1 {
			strings.write_string(&b, "    }\n")
		} else {
			strings.write_string(&b, "    },\n")
		}
	}
	strings.write_string(&b, "  ],\n")

	// Planned trajectory
	fmt.sbprintf(&b, "  \"planned_path\": [\n")
	for i in 0 ..< len(trajectory_waypoints) {
		pt := trajectory_waypoints[i]
		fmt.sbprintf(&b, "    [%.3f, %.3f, %.3f]", pt.x, pt.y, pt.z)
		if i < len(trajectory_waypoints) - 1 {
			strings.write_string(&b, ",\n")
		} else {
			strings.write_string(&b, "\n")
		}
	}
	strings.write_string(&b, "  ]\n")

	strings.write_string(&b, "}")
	return strings.to_string(b)
}

//
// Serialize EngineConfig to JSON string.
//
json_serialize_config :: proc(cfg: EngineConfig) -> string {
	b := strings.builder_make()
	strings.write_string(&b, "{\n")
	fmt.sbprintf(&b, "  \"voxel_size\": %.4f,\n", cfg.voxel_size)
	fmt.sbprintf(&b, "  \"icp_max_iters\": %d,\n", cfg.icp_max_iters)
	fmt.sbprintf(&b, "  \"icp_tolerance\": %.6f,\n", cfg.icp_tolerance)
	fmt.sbprintf(&b, "  \"esdf_max_dist\": %.2f,\n", cfg.esdf_max_dist)
	fmt.sbprintf(&b, "  \"max_velocity\": %.2f,\n", cfg.max_velocity)
	fmt.sbprintf(&b, "  \"max_acceleration\": %.2f,\n", cfg.max_acceleration)
	fmt.sbprintf(&b, "  \"tracking_gating\": %.2f\n", cfg.tracking_gating)
	strings.write_string(&b, "}\n")
	return strings.to_string(b)
}

//
// Helper extracting numeric value for a given JSON key.
//
json_extract_number :: proc(json_str, key: string) -> (f32, bool) {
	key_pattern := fmt.tprintf("\"%s\"", key)
	idx := strings.index(json_str, key_pattern)
	if idx == -1 do return 0.0, false

	colon_rel := strings.index(json_str[idx:], ":")
	if colon_rel == -1 do return 0.0, false

	start := idx + colon_rel + 1
	sub := strings.trim_space(json_str[start:])

	end := 0
	for end < len(sub) {
		c := sub[end]
		if (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E' {
			end += 1
		} else {
			break
		}
	}

	if end == 0 do return 0.0, false
	num_str := sub[:end]
	val, ok := strconv.parse_f32(num_str)
	return val, ok
}

//
// Parse EngineConfig from JSON string.
//
json_parse_config :: proc(json_str: string, cfg_out: ^EngineConfig) -> bool {
	cfg_out^ = engine_config_default()

	if val, ok := json_extract_number(json_str, "voxel_size"); ok {
		cfg_out.voxel_size = val
	}
	if val, ok := json_extract_number(json_str, "icp_max_iters"); ok {
		cfg_out.icp_max_iters = int(val)
	}
	if val, ok := json_extract_number(json_str, "icp_tolerance"); ok {
		cfg_out.icp_tolerance = val
	}
	if val, ok := json_extract_number(json_str, "esdf_max_dist"); ok {
		cfg_out.esdf_max_dist = val
	}
	if val, ok := json_extract_number(json_str, "max_velocity"); ok {
		cfg_out.max_velocity = val
	}
	if val, ok := json_extract_number(json_str, "max_acceleration"); ok {
		cfg_out.max_acceleration = val
	}
	if val, ok := json_extract_number(json_str, "tracking_gating"); ok {
		cfg_out.tracking_gating = val
	}

	return true
}
