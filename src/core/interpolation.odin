package pelorus_core

import "core:math"

//
// Trajectory pose sample along a continuous-time trajectory.
//
TrajectoryPose :: struct {
	timestamp: f64,
	pose:      Transform3D,
	twist:     Twist6, // Linear and angular body velocity
}

//
// Geodesic shortest path interpolation on SE(3) between two poses at time fraction tau in [0, 1].
// T(tau) = T0 * exp(tau * log(T0^{-1} * T1)).
//
se3_geodesic_interpolate :: proc "contextless" (t0, t1: Transform3D, tau: f32) -> Transform3D {
	tau_clamped := math.clamp(tau, 0.0, 1.0)
	if tau_clamped <= 0.0 do return t0
	if tau_clamped >= 1.0 do return t1

	// Relative transformation: delta_T = T0^{-1} * T1
	inv_t0 := transform3d_inverse(t0)
	delta_t := transform3d_mul(inv_t0, t1)

	// Tangent vector in Lie algebra se(3)
	delta_xi := se3_log(delta_t)

	// Scaled step: tau * delta_xi
	scaled_xi := Twist6{
		rho = delta_xi.rho * tau_clamped,
		phi = delta_xi.phi * tau_clamped,
	}

	// Integrated delta pose
	step_tf := se3_exp(scaled_xi)

	// Result: T0 * step_tf
	return transform3d_mul(t0, step_tf)
}

//
// Evaluate instantaneous 6-DOF body twist along geodesic SE(3) path over time interval dt.
//
se3_geodesic_velocity :: proc "contextless" (t0, t1: Transform3D, dt: f32) -> Twist6 {
	if dt <= 1e-6 {
		return twist6_zero()
	}

	inv_t0 := transform3d_inverse(t0)
	delta_t := transform3d_mul(inv_t0, t1)
	delta_xi := se3_log(delta_t)

	inv_dt := 1.0 / dt
	return Twist6{
		rho = delta_xi.rho * inv_dt,
		phi = delta_xi.phi * inv_dt,
	}
}

//
// Continuous-time pose trajectory buffer with binary search query capability.
//
TrajectoryBuffer :: struct {
	poses:    [dynamic]TrajectoryPose,
	capacity: int,
}

//
// Initialize a trajectory buffer with pre-allocated capacity.
//
trajectory_buffer_init :: proc(buf: ^TrajectoryBuffer, capacity: int) {
	buf.poses = make([dynamic]TrajectoryPose, 0, capacity)
	buf.capacity = capacity
}

//
// Destroy trajectory buffer and release memory.
//
trajectory_buffer_destroy :: proc(buf: ^TrajectoryBuffer) {
	delete(buf.poses)
}

//
// Push a new timestamped pose into the buffer maintaining temporal ordering.
//
trajectory_buffer_push :: proc(buf: ^TrajectoryBuffer, pose: TrajectoryPose) {
	// If capacity exceeded, remove oldest sample
	if len(buf.poses) >= buf.capacity && buf.capacity > 0 {
		ordered_remove(&buf.poses, 0)
	}
	append(&buf.poses, pose)
}

//
// Query interpolated SE(3) pose at arbitrary continuous timestamp t via geodesic interpolation.
// Returns interpolated pose and success boolean (false if timestamp outside range).
//
trajectory_buffer_query :: proc(buf: ^TrajectoryBuffer, timestamp: f64) -> (Transform3D, bool) {
	count := len(buf.poses)
	if count == 0 {
		return transform3d_identity(), false
	}
	if count == 1 || timestamp <= buf.poses[0].timestamp {
		return buf.poses[0].pose, true
	}
	if timestamp >= buf.poses[count - 1].timestamp {
		return buf.poses[count - 1].pose, true
	}

	// Binary search for interval [i, i + 1] such that t_i <= timestamp <= t_{i+1}
	low := 0
	high := count - 1
	for low < high - 1 {
		mid := (low + high) / 2
		if buf.poses[mid].timestamp <= timestamp {
			low = mid
		} else {
			high = mid
		}
	}

	p0 := buf.poses[low]
	p1 := buf.poses[high]
	dt := p1.timestamp - p0.timestamp
	if dt <= 1e-9 {
		return p0.pose, true
	}

	tau := f32((timestamp - p0.timestamp) / dt)
	interpolated := se3_geodesic_interpolate(p0.pose, p1.pose, tau)
	return interpolated, true
}
