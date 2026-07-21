package pelorus_planning

import "core:math"
import core "../core"

//
// Normalize angle to [-PI, PI].
//
normalize_angle :: proc "contextless" (angle: f32) -> f32 {
	a := angle
	for a > math.PI do a -= 2.0 * math.PI
	for a < -math.PI do a += 2.0 * math.PI
	return a
}

//
// Configuration parameters for Pure Pursuit path tracking.
//
PurePursuitConfig :: struct {
	wheelbase:       f32, // Distance between front and rear axles (meters)
	min_lookahead:   f32, // Minimum lookahead distance (meters)
	max_lookahead:   f32, // Maximum lookahead distance (meters)
	lookahead_ratio: f32, // Lookahead gain per unit velocity: Ld = kv * v
	max_steer_angle: f32, // Maximum steering angle limit (radians)
}

//
// Default Pure Pursuit tracking configuration.
//
pure_pursuit_config_default :: proc "contextless" () -> PurePursuitConfig {
	return PurePursuitConfig{
		wheelbase       = 1.5,
		min_lookahead   = 1.0,
		max_lookahead   = 6.0,
		lookahead_ratio = 1.2,
		max_steer_angle = 0.65, // Approx 37 degrees
	}
}

//
// Controller tracking output command.
//
ControllerOutput :: struct {
	steering_angle:    f32, // Commanded front wheel steering angle (rad)
	target_velocity:   f32, // Commanded forward linear velocity (m/s)
	cross_track_error: f32, // Lateral orthogonal deviation from path (meters)
	heading_error:     f32, // Yaw angle deviation from path tangent (rad)
	lookahead_dist:    f32, // Active lookahead distance evaluated
}

//
// Compute dynamic lookahead distance based on current platform speed.
//
pure_pursuit_get_lookahead :: proc "contextless" (cfg: PurePursuitConfig, current_speed: f32) -> f32 {
	speed := math.abs(current_speed)
	ld := cfg.lookahead_ratio * speed
	if ld < cfg.min_lookahead do ld = cfg.min_lookahead
	if ld > cfg.max_lookahead do ld = cfg.max_lookahead
	return ld
}

//
// Find lookahead target point along discrete waypoint sequence.
//
pure_pursuit_find_target :: proc(
	waypoints: []core.Vec3,
	current_pos: core.Vec3,
	lookahead_dist: f32,
) -> (target_pt: core.Vec3, target_idx: int) {
	n := len(waypoints)
	if n == 0 {
		return current_pos, 0
	}
	if n == 1 {
		return waypoints[0], 0
	}

	// 1. Find index of closest waypoint
	closest_idx := 0
	min_dist_sq := core.vec3_distance_sq(current_pos, waypoints[0])

	for i in 1 ..< n {
		d_sq := core.vec3_distance_sq(current_pos, waypoints[i])
		if d_sq < min_dist_sq {
			min_dist_sq = d_sq
			closest_idx = i
		}
	}

	// 2. Search forward along path for point at or beyond lookahead distance
	for i in closest_idx ..< n {
		d := core.vec3_distance(current_pos, waypoints[i])
		if d >= lookahead_dist {
			return waypoints[i], i
		}
	}

	// If all remaining points closer than lookahead, target final waypoint
	return waypoints[n - 1], n - 1
}

//
// Compute Pure Pursuit control command for 2D/3D platform heading.
// vehicle_pos: current 2D/3D location
// vehicle_yaw: current heading angle in radians (counter-clockwise from +X)
//
pure_pursuit_compute :: proc(
	cfg: PurePursuitConfig,
	waypoints: []core.Vec3,
	vehicle_pos: core.Vec3,
	vehicle_yaw: f32,
	current_speed: f32,
	nominal_speed: f32,
) -> ControllerOutput {
	out := ControllerOutput{
		target_velocity = nominal_speed,
	}

	if len(waypoints) < 2 {
		return out
	}

	ld := pure_pursuit_get_lookahead(cfg, current_speed)
	out.lookahead_dist = ld

	target_pt, closest_idx := pure_pursuit_find_target(waypoints, vehicle_pos, ld)

	// Vector from rear axle to lookahead target point in vehicle local frame
	dx := target_pt.x - vehicle_pos.x
	dy := target_pt.y - vehicle_pos.y

	cos_yaw := math.cos(vehicle_yaw)
	sin_yaw := math.sin(vehicle_yaw)

	// Transform to local vehicle coordinate frame (X forward, Y left)
	local_x := cos_yaw * dx + sin_yaw * dy
	local_y := -sin_yaw * dx + cos_yaw * dy

	dist := math.sqrt(local_x * local_x + local_y * local_y)
	if dist < 1e-4 {
		return out
	}

	// Pure Pursuit curvature: kappa = 2 * local_y / (dist^2)
	kappa := 2.0 * local_y / (dist * dist)

	// Bicycle model steering: delta = atan(kappa * wheelbase)
	delta := math.atan(kappa * cfg.wheelbase)

	// Clamp to maximum steering angle limit
	if delta > cfg.max_steer_angle do delta = cfg.max_steer_angle
	if delta < -cfg.max_steer_angle do delta = -cfg.max_steer_angle

	out.steering_angle = delta

	// Compute cross-track and heading error relative to closest segment
	p0 := waypoints[closest_idx]
	next_idx := (closest_idx + 1 < len(waypoints)) ? (closest_idx + 1) : closest_idx
	p1 := waypoints[next_idx]

	seg_vec := p1 - p0
	seg_len := core.vec3_length(seg_vec)
	if seg_len > 1e-4 {
		seg_yaw := math.atan2(seg_vec.y, seg_vec.x)
		out.heading_error = normalize_angle(vehicle_yaw - seg_yaw)

		// 2D cross product for signed lateral error
		lateral_cross := (p1.x - p0.x) * (vehicle_pos.y - p0.y) - (p1.y - p0.y) * (vehicle_pos.x - p0.x)
		out.cross_track_error = lateral_cross / seg_len
	}

	return out
}

//
// Stanley Path Tracking Controller.
// Uses front axle reference with combined heading and cross-track error feedback.
//
StanleyConfig :: struct {
	wheelbase:       f32,
	gain_k:          f32, // Cross-track error gain
	softening_k_soft: f32, // Velocity softening constant (prevents division by zero)
	max_steer_angle: f32,
}

//
// Default Stanley controller configuration.
//
stanley_config_default :: proc "contextless" () -> StanleyConfig {
	return StanleyConfig{
		wheelbase        = 1.5,
		gain_k           = 1.2,
		softening_k_soft = 0.5,
		max_steer_angle  = 0.65,
	}
}

//
// Compute Stanley control steering angle.
// delta = heading_error + atan2(k * cross_track_error, v + k_soft)
//
stanley_compute :: proc(
	cfg: StanleyConfig,
	waypoints: []core.Vec3,
	front_axle_pos: core.Vec3,
	vehicle_yaw: f32,
	current_speed: f32,
	nominal_speed: f32,
) -> ControllerOutput {
	out := ControllerOutput{
		target_velocity = nominal_speed,
	}

	n := len(waypoints)
	if n < 2 do return out

	// Find closest waypoint to front axle
	closest_idx := 0
	min_dist_sq := core.vec3_distance_sq(front_axle_pos, waypoints[0])

	for i in 1 ..< n {
		d_sq := core.vec3_distance_sq(front_axle_pos, waypoints[i])
		if d_sq < min_dist_sq {
			min_dist_sq = d_sq
			closest_idx = i
		}
	}

	// Segment points
	p0 := waypoints[closest_idx]
	next_idx := (closest_idx + 1 < n) ? (closest_idx + 1) : closest_idx
	p1 := waypoints[next_idx]

	seg_vec := p1 - p0
	seg_len := core.vec3_length(seg_vec)
	if seg_len < 1e-4 {
		return out
	}

	// Path tangent yaw
	path_yaw := math.atan2(seg_vec.y, seg_vec.x)

	// Heading error theta_e = yaw_path - yaw_vehicle
	heading_err := normalize_angle(path_yaw - vehicle_yaw)
	out.heading_error = heading_err

	// Signed cross-track error e_y: positive if platform is left of path
	lateral_cross := (p1.x - p0.x) * (front_axle_pos.y - p0.y) - (p1.y - p0.y) * (front_axle_pos.x - p0.x)
	e_y := lateral_cross / seg_len
	out.cross_track_error = e_y

	// Stanley steering control law
	steer_cross := math.atan2(cfg.gain_k * e_y, current_speed + cfg.softening_k_soft)
	delta := heading_err + steer_cross

	// Clamp
	if delta > cfg.max_steer_angle do delta = cfg.max_steer_angle
	if delta < -cfg.max_steer_angle do delta = -cfg.max_steer_angle

	out.steering_angle = delta
	return out
}

//
// Kinematic bicycle vehicle state representation.
//
BicycleState :: struct {
	pos:   core.Vec3,
	yaw:   f32,
	speed: f32,
}

//
// Integrate discrete forward step of kinematic bicycle model.
// dt: time step in seconds
//
bicycle_model_step :: proc "contextless" (
	state: ^BicycleState,
	steering_angle: f32,
	target_speed: f32,
	wheelbase: f32,
	dt: f32,
) {
	// Simple proportional speed tracking
	speed_error := target_speed - state.speed
	acc := speed_error * 2.0
	state.speed += acc * dt
	if state.speed < 0.0 do state.speed = 0.0

	// Kinematic propagation
	state.pos.x += state.speed * math.cos(state.yaw) * dt
	state.pos.y += state.speed * math.sin(state.yaw) * dt

	yaw_rate := (state.speed / wheelbase) * math.tan(steering_angle)
	state.yaw = normalize_angle(state.yaw + yaw_rate * dt)
}
