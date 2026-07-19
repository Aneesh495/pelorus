package pelorus_planning

import "core:math"
import core "../core"

//
// 1D Quintic Polynomial: p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
//
Quintic1D :: struct {
	c0, c1, c2, c3, c4, c5: f32,
}

//
// Evaluate 1D polynomial position at time t in [0, T].
//
quintic1d_eval_pos :: proc(q: Quintic1D, t: f32) -> f32 {
	t2 := t * t
	t3 := t2 * t
	t4 := t3 * t
	t5 := t4 * t
	return q.c0 + q.c1 * t + q.c2 * t2 + q.c3 * t3 + q.c4 * t4 + q.c5 * t5
}

//
// Evaluate 1D polynomial velocity at time t in [0, T].
//
quintic1d_eval_vel :: proc(q: Quintic1D, t: f32) -> f32 {
	t2 := t * t
	t3 := t2 * t
	t4 := t3 * t
	return q.c1 + 2.0 * q.c2 * t + 3.0 * q.c3 * t2 + 4.0 * q.c4 * t3 + 5.0 * q.c5 * t4
}

//
// Evaluate 1D polynomial acceleration at time t in [0, T].
//
quintic1d_eval_acc :: proc(q: Quintic1D, t: f32) -> f32 {
	t2 := t * t
	t3 := t2 * t
	return 2.0 * q.c2 + 6.0 * q.c3 * t + 12.0 * q.c4 * t2 + 20.0 * q.c5 * t3
}

//
// Evaluate 1D polynomial jerk at time t in [0, T].
//
quintic1d_eval_jerk :: proc(q: Quintic1D, t: f32) -> f32 {
	t2 := t * t
	return 6.0 * q.c3 + 24.0 * q.c4 * t + 60.0 * q.c5 * t2
}

//
// Solve 1D minimum-jerk boundary value problem over duration T.
// Boundary conditions:
//   at t=0: p0, v0, a0
//   at t=T: p1, v1, a1
//
quintic1d_solve :: proc(
	p0, v0, a0: f32,
	p1, v1, a1: f32,
	T: f32,
) -> Quintic1D {
	if T <= 1e-4 {
		return Quintic1D{c0 = p0, c1 = v0, c2 = 0.5 * a0, c3 = 0, c4 = 0, c5 = 0}
	}

	T2 := T * T
	T3 := T2 * T
	T4 := T3 * T
	T5 := T4 * T

	c0 := p0
	c1 := v0
	c2 := 0.5 * a0

	delta_p := p1 - c0 - c1 * T - c2 * T2
	delta_v := v1 - c1 - 2.0 * c2 * T
	delta_a := a1 - 2.0 * c2

	c3 := (10.0 * delta_p / T3) - (4.0 * delta_v / T2) + (0.5 * delta_a / T)
	c4 := (-15.0 * delta_p / T4) + (7.0 * delta_v / T3) - (delta_a / T2)
	c5 := (6.0 * delta_p / T5) - (3.0 * delta_v / T4) + (0.5 * delta_a / T3)

	return Quintic1D{
		c0 = c0,
		c1 = c1,
		c2 = c2,
		c3 = c3,
		c4 = c4,
		c5 = c5,
	}
}

//
// 3D Quintic Polynomial Spline Segment.
//
QuinticSegment3D :: struct {
	x:          Quintic1D,
	y:          Quintic1D,
	z:          Quintic1D,
	t_start:    f64,
	duration:   f32,
}

//
// Sample 3D position at local time t.
//
quintic_segment_eval_pos :: proc(seg: QuinticSegment3D, t: f32) -> core.Vec3 {
	return core.Vec3{
		quintic1d_eval_pos(seg.x, t),
		quintic1d_eval_pos(seg.y, t),
		quintic1d_eval_pos(seg.z, t),
	}
}

//
// Sample 3D velocity at local time t.
//
quintic_segment_eval_vel :: proc(seg: QuinticSegment3D, t: f32) -> core.Vec3 {
	return core.Vec3{
		quintic1d_eval_vel(seg.x, t),
		quintic1d_eval_vel(seg.y, t),
		quintic1d_eval_vel(seg.z, t),
	}
}

//
// Sample 3D acceleration at local time t.
//
quintic_segment_eval_acc :: proc(seg: QuinticSegment3D, t: f32) -> core.Vec3 {
	return core.Vec3{
		quintic1d_eval_acc(seg.x, t),
		quintic1d_eval_acc(seg.y, t),
		quintic1d_eval_acc(seg.z, t),
	}
}

//
// Sample 3D jerk at local time t.
//
quintic_segment_eval_jerk :: proc(seg: QuinticSegment3D, t: f32) -> core.Vec3 {
	return core.Vec3{
		quintic1d_eval_jerk(seg.x, t),
		quintic1d_eval_jerk(seg.y, t),
		quintic1d_eval_jerk(seg.z, t),
	}
}

//
// Full trajectory composed of sequence of continuous quintic spline segments.
//
QuinticTrajectory3D :: struct {
	segments:    [dynamic]QuinticSegment3D,
	total_time:  f64,
	owns_memory: bool,
}

//
// Initialize empty trajectory.
//
quintic_trajectory_init :: proc(traj: ^QuinticTrajectory3D) {
	traj.segments = make([dynamic]QuinticSegment3D, 0, 16)
	traj.total_time = 0.0
	traj.owns_memory = true
}

//
// Destroy trajectory.
//
quintic_trajectory_destroy :: proc(traj: ^QuinticTrajectory3D) {
	if traj.owns_memory {
		delete(traj.segments)
	}
	traj.segments = nil
}

//
// Clear trajectory segments.
//
quintic_trajectory_clear :: proc(traj: ^QuinticTrajectory3D) {
	clear(&traj.segments)
	traj.total_time = 0.0
}

//
// Generate multi-segment Minimum-Jerk trajectory through discrete 3D waypoints.
// Applies continuous intermediate velocity heuristics matching Euclidean direction.
//
generate_minimum_jerk_trajectory :: proc(
	waypoints: []core.Vec3,
	v0, a0: core.Vec3,
	v_final, a_final: core.Vec3,
	nominal_speed: f32,
	traj_out: ^QuinticTrajectory3D,
) -> bool {
	n := len(waypoints)
	if n < 2 do return false

	quintic_trajectory_clear(traj_out)

	// Allocate segment velocities at intermediate knots
	velocities := make([]core.Vec3, n)
	defer delete(velocities)

	velocities[0] = v0
	velocities[n - 1] = v_final

	for i in 1 ..< (n - 1) {
		prev := waypoints[i - 1]
		next := waypoints[i + 1]
		dir := next - prev
		len_dir := core.vec3_length(dir)
		if len_dir > 1e-4 {
			velocities[i] = dir * (nominal_speed / len_dir)
		} else {
			velocities[i] = core.Vec3{0, 0, 0}
		}
	}

	t_accum := f64(0.0)

	for i in 0 ..< (n - 1) {
		p0 := waypoints[i]
		p1 := waypoints[i + 1]
		dist := core.vec3_distance(p0, p1)
		duration := dist / nominal_speed
		if duration < 0.1 do duration = 0.1

		curr_v0 := velocities[i]
		curr_v1 := velocities[i + 1]
		curr_a0 := (i == 0) ? a0 : core.Vec3{0, 0, 0}
		curr_a1 := (i == n - 2) ? a_final : core.Vec3{0, 0, 0}

		seg := QuinticSegment3D{
			x = quintic1d_solve(p0.x, curr_v0.x, curr_a0.x, p1.x, curr_v1.x, curr_a1.x, duration),
			y = quintic1d_solve(p0.y, curr_v0.y, curr_a0.y, p1.y, curr_v1.y, curr_a1.y, duration),
			z = quintic1d_solve(p0.z, curr_v0.z, curr_a0.z, p1.z, curr_v1.z, curr_a1.z, duration),
			t_start = t_accum,
			duration = duration,
		}

		append(&traj_out.segments, seg)
		t_accum += f64(duration)
	}

	traj_out.total_time = t_accum
	return true
}

//
// Sample state along full multi-segment trajectory at global timestamp t_global.
//
quintic_trajectory_sample :: proc(
	traj: ^QuinticTrajectory3D,
	t_global: f64,
) -> (pos: core.Vec3, vel: core.Vec3, acc: core.Vec3, yaw: f32) {
	if len(traj.segments) == 0 {
		return core.Vec3{0, 0, 0}, core.Vec3{0, 0, 0}, core.Vec3{0, 0, 0}, 0.0
	}

	// Clamp to trajectory domain
	if t_global <= 0.0 {
		first := traj.segments[0]
		pos = quintic_segment_eval_pos(first, 0.0)
		vel = quintic_segment_eval_vel(first, 0.0)
		acc = quintic_segment_eval_acc(first, 0.0)
		yaw = math.atan2(vel.y, vel.x)
		return
	}

	if t_global >= traj.total_time {
		last := traj.segments[len(traj.segments) - 1]
		pos = quintic_segment_eval_pos(last, last.duration)
		vel = quintic_segment_eval_vel(last, last.duration)
		acc = quintic_segment_eval_acc(last, last.duration)
		yaw = math.atan2(vel.y, vel.x)
		return
	}

	// Find active segment
	for seg in traj.segments {
		if t_global >= seg.t_start && t_global <= (seg.t_start + f64(seg.duration)) {
			t_local := f32(t_global - seg.t_start)
			pos = quintic_segment_eval_pos(seg, t_local)
			vel = quintic_segment_eval_vel(seg, t_local)
			acc = quintic_segment_eval_acc(seg, t_local)
			speed_xy := math.sqrt(vel.x * vel.x + vel.y * vel.y)
			if speed_xy > 1e-3 {
				yaw = math.atan2(vel.y, vel.x)
			} else {
				yaw = 0.0
			}
			return
		}
	}

	// Fallback to last segment end
	last := traj.segments[len(traj.segments) - 1]
	pos = quintic_segment_eval_pos(last, last.duration)
	vel = quintic_segment_eval_vel(last, last.duration)
	acc = quintic_segment_eval_acc(last, last.duration)
	yaw = math.atan2(vel.y, vel.x)
	return
}

//
// Verify kinodynamic feasibility against velocity and acceleration limits.
//
quintic_trajectory_check_limits :: proc(
	traj: ^QuinticTrajectory3D,
	limits: KinodynamicLimits,
	sample_dt: f32 = 0.05,
) -> bool {
	if len(traj.segments) == 0 do return false

	t := f64(0.0)
	dt := f64(sample_dt)
	if dt <= 0.0 do dt = 0.05

	for t <= traj.total_time {
		_, vel, acc, _ := quintic_trajectory_sample(traj, t)
		speed := core.vec3_length(vel)
		acc_mag := core.vec3_length(acc)

		if speed > limits.max_velocity * 1.05 do return false
		if acc_mag > limits.max_acceleration * 1.05 do return false

		t += dt
	}

	return true
}
