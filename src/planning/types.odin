package pelorus_planning

import core "../core"

//
// Kinodynamic limits for trajectory optimization and motion planning.
//
KinodynamicLimits :: struct {
	max_velocity:     f32, // Maximum speed limit (m/s)
	max_acceleration: f32, // Maximum linear acceleration (m/s^2)
	max_jerk:         f32, // Maximum jerk derivative (m/s^3)
	max_curvature:    f32, // Maximum path curvature kappa (rad/m)
	safety_margin_m:  f32, // Buffer distance from obstacle boundaries (meters)
}

//
// Default kinodynamic parameters for autonomous mobile platforms.
//
kinodynamic_limits_default :: proc "contextless" () -> KinodynamicLimits {
	return KinodynamicLimits{
		max_velocity     = 4.0,
		max_acceleration = 2.5,
		max_jerk         = 8.0,
		max_curvature    = 0.6,
		safety_margin_m  = 0.3,
	}
}

//
// 3D trajectory state sample at continuous timestamp.
//
Waypoint3D :: struct {
	position:        core.Vec3,
	velocity:        core.Vec3,
	acceleration:    core.Vec3,
	jerk:            core.Vec3,
	yaw:             f32,
	curvature:       f32,
	time_from_start: f32,
}

//
// Analytical 5th-order (quintic) polynomial trajectory segment in 3D:
// p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5.
// Guarantees C^2 continuous position, velocity, and acceleration profiles.
//
QuinticCoeffs3D :: struct {
	c0: core.Vec3,
	c1: core.Vec3,
	c2: core.Vec3,
	c3: core.Vec3,
	c4: core.Vec3,
	c5: core.Vec3,
}

//
// Individual piecewise polynomial segment within full trajectory.
//
TrajectorySegment :: struct {
	start_time: f32,
	duration:   f32,
	coeffs:     QuinticCoeffs3D,
}

//
// Complete piecewise continuous trajectory.
//
FullTrajectory :: struct {
	segments:       [dynamic]TrajectorySegment,
	total_duration: f32,
	total_length:   f32,
	owns_memory:    bool,
}

//
// Initialize FullTrajectory.
//
trajectory_init :: proc(traj: ^FullTrajectory) {
	traj.segments = make([dynamic]TrajectorySegment, 0, 32)
	traj.total_duration = 0.0
	traj.total_length = 0.0
	traj.owns_memory = true
}

//
// Destroy FullTrajectory.
//
trajectory_destroy :: proc(traj: ^FullTrajectory) {
	if traj.owns_memory {
		delete(traj.segments)
	}
	traj.segments = nil
	traj.total_duration = 0.0
	traj.total_length = 0.0
}

//
// Evaluate position, velocity, and acceleration along a quintic polynomial segment at local time t.
//
quintic_evaluate :: proc "contextless" (coeffs: QuinticCoeffs3D, t: f32) -> (p: core.Vec3, v: core.Vec3, a: core.Vec3) {
	t2 := t * t
	t3 := t2 * t
	t4 := t3 * t
	t5 := t4 * t

	// p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
	p = coeffs.c0 + coeffs.c1 * t + coeffs.c2 * t2 + coeffs.c3 * t3 + coeffs.c4 * t4 + coeffs.c5 * t5

	// v(t) = c1 + 2*c2*t + 3*c3*t^2 + 4*c4*t^3 + 5*c5*t^4
	v = coeffs.c1 + coeffs.c2 * (2.0 * t) + coeffs.c3 * (3.0 * t2) + coeffs.c4 * (4.0 * t3) + coeffs.c5 * (5.0 * t4)

	// a(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3
	a = coeffs.c2 * 2.0 + coeffs.c3 * (6.0 * t) + coeffs.c4 * (12.0 * t2) + coeffs.c5 * (20.0 * t3)

	return
}

//
// Sample FullTrajectory at continuous query time t in [0, total_duration].
//
trajectory_sample :: proc "contextless" (traj: ^FullTrajectory, t_query: f32) -> Waypoint3D {
	n := len(traj.segments)
	if n == 0 {
		return Waypoint3D{}
	}

	if t_query <= 0.0 {
		p, v, a := quintic_evaluate(traj.segments[0].coeffs, 0.0)
		return Waypoint3D{
			position     = p,
			velocity     = v,
			acceleration = a,
			time_from_start = 0.0,
		}
	}

	if t_query >= traj.total_duration {
		last_seg := traj.segments[n - 1]
		p, v, a := quintic_evaluate(last_seg.coeffs, last_seg.duration)
		return Waypoint3D{
			position     = p,
			velocity     = v,
			acceleration = a,
			time_from_start = traj.total_duration,
		}
	}

	// Find containing segment
	for seg in traj.segments {
		if t_query >= seg.start_time && t_query <= (seg.start_time + seg.duration) {
			local_t := t_query - seg.start_time
			p, v, a := quintic_evaluate(seg.coeffs, local_t)
			return Waypoint3D{
				position     = p,
				velocity     = v,
				acceleration = a,
				time_from_start = t_query,
			}
		}
	}

	return Waypoint3D{}
}
