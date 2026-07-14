package pelorus_planning

import "core:math"
import core "../core"
import mapping "../mapping"

//
// 3D Convex Safe Flight Corridor Box.
// Represents an axis-aligned convex polytope certified obstacle-free by ESDF queries.
//
CorridorBox :: struct {
	bounds: core.AABB,
}

//
// Safe Flight Corridor Generator.
// Inflates collision-free convex polyhedral boxes along a discrete guide path.
//
CorridorGenerator :: struct {
	safety_margin: f32, // Minimum distance clearance from obstacle surfaces (meters)
	step_size:     f32, // Box expansion step size (meters)
	max_box_size:  f32, // Upper bound on corridor box dimensions
}

//
// Initialize CorridorGenerator.
//
corridor_generator_init :: proc(
	gen: ^CorridorGenerator,
	safety_margin: f32 = 0.2,
	step_size: f32 = 0.1,
	max_box_size: f32 = 6.0,
) {
	gen.safety_margin = safety_margin
	gen.step_size = step_size
	gen.max_box_size = max_box_size
}

//
// Test if an AABB is completely certified free of obstacles in the ESDF map.
// Evaluates clearance distance across box corners and center.
//
corridor_box_is_free :: proc(
	esdf: ^mapping.EsdfMap,
	box: core.AABB,
	margin: f32,
) -> bool {
	// Sample 8 corners + center point
	corners := [9]core.Vec3{
		box.min_pt,
		core.Vec3{box.max_pt.x, box.min_pt.y, box.min_pt.z},
		core.Vec3{box.min_pt.x, box.max_pt.y, box.min_pt.z},
		core.Vec3{box.min_pt.x, box.min_pt.z, box.max_pt.z},
		core.Vec3{box.max_pt.x, box.max_pt.y, box.min_pt.z},
		core.Vec3{box.max_pt.x, box.min_pt.y, box.max_pt.z},
		core.Vec3{box.min_pt.x, box.max_pt.y, box.max_pt.z},
		box.max_pt,
		core.aabb_center(box),
	}

	for p in corners {
		d, _, ok := mapping.esdf_query_distance_and_gradient(esdf, p)
		if !ok || d < margin {
			return false
		}
	}

	return true
}

//
// Inflate a collision-free convex box around a seed point inside the ESDF map.
// Expands +X, -X, +Y, -Y, +Z, -Z iteratively until reaching obstacle boundaries.
//
corridor_inflate_box :: proc(
	gen: ^CorridorGenerator,
	esdf: ^mapping.EsdfMap,
	seed_pt: core.Vec3,
) -> (CorridorBox, bool) {
	margin := gen.safety_margin
	step := gen.step_size
	max_dim := gen.max_box_size

	// Check if seed itself is collision free
	seed_d, _, ok := mapping.esdf_query_distance_and_gradient(esdf, seed_pt)
	if !ok || seed_d < margin {
		return CorridorBox{}, false
	}

	half_step := step * 0.5
	box := core.aabb_make(
		seed_pt - core.Vec3{half_step, half_step, half_step},
		seed_pt + core.Vec3{half_step, half_step, half_step},
	)

	// Direction expansion flags: [+X, -X, +Y, -Y, +Z, -Z]
	can_expand := [6]bool{true, true, true, true, true, true}

	max_iters :: 40
	for iter := 0; iter < max_iters; iter += 1 {
		any_expanded := false

		// 1. Expand +X
		if can_expand[0] {
			test_box := box
			test_box.max_pt.x += step
			if (test_box.max_pt.x - test_box.min_pt.x) <= max_dim && corridor_box_is_free(esdf, test_box, margin) {
				box = test_box
				any_expanded = true
			} else {
				can_expand[0] = false
			}
		}

		// 2. Expand -X
		if can_expand[1] {
			test_box := box
			test_box.min_pt.x -= step
			if (test_box.max_pt.x - test_box.min_pt.x) <= max_dim && corridor_box_is_free(esdf, test_box, margin) {
				box = test_box
				any_expanded = true
			} else {
				can_expand[1] = false
			}
		}

		// 3. Expand +Y
		if can_expand[2] {
			test_box := box
			test_box.max_pt.y += step
			if (test_box.max_pt.y - test_box.min_pt.y) <= max_dim && corridor_box_is_free(esdf, test_box, margin) {
				box = test_box
				any_expanded = true
			} else {
				can_expand[2] = false
			}
		}

		// 4. Expand -Y
		if can_expand[3] {
			test_box := box
			test_box.min_pt.y -= step
			if (test_box.max_pt.y - test_box.min_pt.y) <= max_dim && corridor_box_is_free(esdf, test_box, margin) {
				box = test_box
				any_expanded = true
			} else {
				can_expand[3] = false
			}
		}

		// 5. Expand +Z
		if can_expand[4] {
			test_box := box
			test_box.max_pt.z += step
			if (test_box.max_pt.z - test_box.min_pt.z) <= max_dim && corridor_box_is_free(esdf, test_box, margin) {
				box = test_box
				any_expanded = true
			} else {
				can_expand[4] = false
			}
		}

		// 6. Expand -Z
		if can_expand[5] {
			test_box := box
			test_box.min_pt.z -= step
			if (test_box.max_pt.z - test_box.min_pt.z) <= max_dim && corridor_box_is_free(esdf, test_box, margin) {
				box = test_box
				any_expanded = true
			} else {
				can_expand[5] = false
			}
		}

		if !any_expanded do break
	}

	return CorridorBox{bounds = box}, true
}

//
// Generate overlapping Safe Flight Corridor along a discrete guide path.
//
corridor_generate_path :: proc(
	gen: ^CorridorGenerator,
	esdf: ^mapping.EsdfMap,
	path_waypoints: []core.Vec3,
	corridors_out: ^[dynamic]CorridorBox,
) -> int {
	if len(path_waypoints) == 0 do return 0

	for pt in path_waypoints {
		c_box, ok := corridor_inflate_box(gen, esdf, pt)
		if ok {
			append(corridors_out, c_box)
		}
	}

	return len(corridors_out)
}
