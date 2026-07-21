package pelorus_tests

import "core:math"
import "core:testing"
import core "../src/core"
import mapping "../src/mapping"
import planning "../src/planning"

//
// Test 1D and 3D Quintic Polynomial Trajectory boundary conditions.
//
@(test)
test_quintic_polynomial_boundary_conditions :: proc(t: ^testing.T) {
	p0 := core.Vec3{1.0, -2.0, 0.5}
	v0 := core.Vec3{0.2, 0.0, -0.1}
	a0 := core.Vec3{0.0, 0.1, 0.0}

	p1 := core.Vec3{5.0, 3.0, 2.0}
	v1 := core.Vec3{1.0, 0.5, 0.0}
	a1 := core.Vec3{0.0, -0.2, 0.0}

	duration := f32(4.0)

	seg := planning.QuinticSegment3D{
		x = planning.quintic1d_solve(p0.x, v0.x, a0.x, p1.x, v1.x, a1.x, duration),
		y = planning.quintic1d_solve(p0.y, v0.y, a0.y, p1.y, v1.y, a1.y, duration),
		z = planning.quintic1d_solve(p0.z, v0.z, a0.z, p1.z, v1.z, a1.z, duration),
		t_start = 0.0,
		duration = duration,
	}

	// Test boundary conditions at t = 0
	eval_p0 := planning.quintic_segment_eval_pos(seg, 0.0)
	eval_v0 := planning.quintic_segment_eval_vel(seg, 0.0)
	eval_a0 := planning.quintic_segment_eval_acc(seg, 0.0)

	testing.expect(t, core.vec3_distance(eval_p0, p0) < 1e-4, "Start position boundary condition failed")
	testing.expect(t, core.vec3_distance(eval_v0, v0) < 1e-4, "Start velocity boundary condition failed")
	testing.expect(t, core.vec3_distance(eval_a0, a0) < 1e-4, "Start acceleration boundary condition failed")

	// Test boundary conditions at t = T
	eval_p1 := planning.quintic_segment_eval_pos(seg, duration)
	eval_v1 := planning.quintic_segment_eval_vel(seg, duration)
	eval_a1 := planning.quintic_segment_eval_acc(seg, duration)

	testing.expect(t, core.vec3_distance(eval_p1, p1) < 1e-4, "End position boundary condition failed")
	testing.expect(t, core.vec3_distance(eval_v1, v1) < 1e-4, "End velocity boundary condition failed")
	testing.expect(t, core.vec3_distance(eval_a1, a1) < 1e-4, "End acceleration boundary condition failed")
}

//
// Test Multi-segment Minimum Jerk Trajectory generation.
//
@(test)
test_minimum_jerk_multi_segment :: proc(t: ^testing.T) {
	waypoints := []core.Vec3{
		{0.0, 0.0, 1.0},
		{2.0, 1.0, 1.2},
		{4.0, 0.0, 1.5},
		{6.0, 2.0, 2.0},
	}

	traj: planning.QuinticTrajectory3D
	planning.quintic_trajectory_init(&traj)
	defer planning.quintic_trajectory_destroy(&traj)

	v0 := core.Vec3{0, 0, 0}
	a0 := core.Vec3{0, 0, 0}
	vf := core.Vec3{0, 0, 0}
	af := core.Vec3{0, 0, 0}

	ok := planning.generate_minimum_jerk_trajectory(waypoints, v0, a0, vf, af, 2.0, &traj)
	testing.expect(t, ok, "Trajectory generation should succeed")
	testing.expect(t, len(traj.segments) == 3, "Should produce 3 spline segments")
	testing.expect(t, traj.total_time > 1.0, "Total duration should be positive")

	// Check start and end sampling
	p_start, v_start, _, _ := planning.quintic_trajectory_sample(&traj, 0.0)
	p_end, v_end, _, _ := planning.quintic_trajectory_sample(&traj, traj.total_time)

	testing.expect(t, core.vec3_distance(p_start, waypoints[0]) < 1e-4, "Start sample matches first waypoint")
	testing.expect(t, core.vec3_distance(p_end, waypoints[3]) < 1e-4, "End sample matches last waypoint")
	testing.expect(t, core.vec3_length(v_start) < 1e-3, "Start velocity matches initial zero constraint")
	testing.expect(t, core.vec3_length(v_end) < 1e-3, "End velocity matches final zero constraint")
}

//
// Test Safe Flight Corridor inflation.
//
@(test)
test_safe_flight_corridor_inflation :: proc(t: ^testing.T) {
	// Create small ESDF map
	esdf: mapping.EsdfMap
	mapping.esdf_map_init(&esdf, 0.2, 32, 5.0)
	defer mapping.esdf_map_destroy(&esdf)

	// Populate ESDF cells with positive distances
	for i in 0 ..< len(esdf.voxels) {
		esdf.voxels[i].distance = 1.5
		esdf.voxels[i].observed = true
	}

	path := []core.Vec3{
		{0.0, 0.0, 0.5},
		{0.5, 0.0, 0.5},
		{1.0, 0.0, 0.5},
	}

	gen: planning.CorridorGenerator
	planning.corridor_generator_init(&gen, 0.2, 0.1, 4.0)

	corridors := make([dynamic]planning.CorridorBox)
	defer delete(corridors)

	count := planning.corridor_generate_path(&gen, &esdf, path, &corridors)
	testing.expect(t, count > 0, "Corridor inflation should generate at least one box")

	// Verify all guide waypoints are contained inside generated corridor boxes
	for i in 0 ..< count {
		pt := path[i]
		box := corridors[i].bounds
		testing.expect(t, core.aabb_contains_point(box, pt), "Corridor box should contain guide waypoint")
	}
}

//
// Test Pure Pursuit path tracking convergence.
//
@(test)
test_path_tracking_controller_convergence :: proc(t: ^testing.T) {
	// Straight line path along X axis
	waypoints := []core.Vec3{
		{0.0, 0.0, 0.0},
		{2.0, 0.0, 0.0},
		{4.0, 0.0, 0.0},
		{6.0, 0.0, 0.0},
		{8.0, 0.0, 0.0},
		{10.0, 0.0, 0.0},
	}

	// Platform initialized with lateral offset of 0.8 meters
	state := planning.BicycleState{
		pos   = core.Vec3{0.0, 0.8, 0.0},
		yaw   = 0.0, // Parallel to path
		speed = 1.0,
	}

	cfg := planning.pure_pursuit_config_default()
	nominal_speed := f32(2.0)
	dt := f32(0.05)

	initial_error := math.abs(state.pos.y)

	// Simulate for 80 steps (4 seconds)
	for _ in 0 ..< 80 {
		cmd := planning.pure_pursuit_compute(cfg, waypoints, state.pos, state.yaw, state.speed, nominal_speed)
		planning.bicycle_model_step(&state, cmd.steering_angle, cmd.target_velocity, cfg.wheelbase, dt)
	}

	final_error := math.abs(state.pos.y)
	testing.expect(t, final_error < initial_error * 0.3, "Pure pursuit should substantially reduce cross-track error")
}
