package pelorus_planning

import "core:math"
import core "../core"
import mapping "../mapping"

//
// RRT* search tree node.
//
RrtNode :: struct {
	position: core.Vec3,
	cost:     f32, // Cumulative path cost from start
	parent:   int, // Index into node pool (-1 for root)
}

//
// Kinodynamic RRT* Motion Planner.
// Searches asymptotically optimal collision-free paths in 3D Euclidean configuration space.
//
RrtStarPlanner :: struct {
	max_iterations: int,
	step_size:      f32, // Maximum expansion step length (meters)
	search_radius:  f32, // Rewiring neighborhood radius
	goal_bias:      f32, // Probability in [0, 1] of sampling goal directly
	safety_margin:  f32,
	nodes:          [dynamic]RrtNode,
	owns_memory:    bool,
}

//
// Initialize RrtStarPlanner.
//
rrt_star_init :: proc(
	planner: ^RrtStarPlanner,
	step_size: f32 = 0.5,
	search_radius: f32 = 1.2,
	max_iterations: int = 1000,
	safety_margin: f32 = 0.3,
) {
	planner.max_iterations = max_iterations
	planner.step_size = step_size
	planner.search_radius = search_radius
	planner.goal_bias = 0.15
	planner.safety_margin = safety_margin
	planner.nodes = make([dynamic]RrtNode, 0, max_iterations + 10)
	planner.owns_memory = true
}

//
// Destroy planner.
//
rrt_star_destroy :: proc(planner: ^RrtStarPlanner) {
	if planner.owns_memory {
		delete(planner.nodes)
	}
	planner.nodes = nil
}

//
// Simple deterministic pseudo-random 3D point generator inside bounding volume.
//
rrt_sample_random_point :: proc(
	bounds: core.AABB,
	goal: core.Vec3,
	goal_bias: f32,
	rng_state: ^u64,
) -> core.Vec3 {
	rng_state^ = rng_state^ * 6364136223846793005 + 1442695040888963407
	r_bias := f32(rng_state^ & 0xFFFF) / 65535.0
	if r_bias < goal_bias {
		return goal
	}

	rng_state^ = rng_state^ * 6364136223846793005 + 1442695040888963407
	rx := f32(rng_state^ & 0xFFFF) / 65535.0
	rng_state^ = rng_state^ * 6364136223846793005 + 1442695040888963407
	ry := f32(rng_state^ & 0xFFFF) / 65535.0
	rng_state^ = rng_state^ * 6364136223846793005 + 1442695040888963407
	rz := f32(rng_state^ & 0xFFFF) / 65535.0

	return core.Vec3{
		bounds.min_pt.x + rx * (bounds.max_pt.x - bounds.min_pt.x),
		bounds.min_pt.y + ry * (bounds.max_pt.y - bounds.min_pt.y),
		bounds.min_pt.z + rz * (bounds.max_pt.z - bounds.min_pt.z),
	}
}

//
// Check collision along line segment between p1 and p2 in ESDF map.
//
rrt_is_segment_free :: proc(
	esdf: ^mapping.EsdfMap,
	p1, p2: core.Vec3,
	margin: f32,
) -> bool {
	dist := core.vec3_distance(p1, p2)
	steps := int(math.ceil(dist / (esdf.voxel_size * 0.5)))
	if steps < 2 do steps = 2

	inv_steps := 1.0 / f32(steps)
	for s in 0 ..= steps {
		t := f32(s) * inv_steps
		p := core.vec3_lerp(p1, p2, t)
		d, _, ok := mapping.esdf_query_distance_and_gradient(esdf, p)
		if !ok || d < margin {
			return false
		}
	}

	return true
}

//
// Plan collision-free path from start to goal through ESDF map.
// Appends path waypoints to path_out. Returns true if path found.
//
rrt_star_plan :: proc(
	planner: ^RrtStarPlanner,
	esdf: ^mapping.EsdfMap,
	start: core.Vec3,
	goal: core.Vec3,
	search_bounds: core.AABB,
	path_out: ^[dynamic]core.Vec3,
) -> bool {
	clear(&planner.nodes)

	// Check endpoints clearance
	start_d, _, ok_start := mapping.esdf_query_distance_and_gradient(esdf, start)
	goal_d, _, ok_goal := mapping.esdf_query_distance_and_gradient(esdf, goal)
	if !ok_start || start_d < planner.safety_margin do return false
	if !ok_goal || goal_d < planner.safety_margin do return false

	// Add start root node
	append(&planner.nodes, RrtNode{
		position = start,
		cost     = 0.0,
		parent   = -1,
	})

	best_goal_node := -1
	best_goal_cost := f32(1e9)
	rng_state := u64(42)

	for iter in 0 ..< planner.max_iterations {
		sample_pt := rrt_sample_random_point(search_bounds, goal, planner.goal_bias, &rng_state)

		// 1. Find nearest node in tree
		nearest_idx := 0
		nearest_dist_sq := core.vec3_distance_sq(planner.nodes[0].position, sample_pt)

		for i in 1 ..< len(planner.nodes) {
			d_sq := core.vec3_distance_sq(planner.nodes[i].position, sample_pt)
			if d_sq < nearest_dist_sq {
				nearest_dist_sq = d_sq
				nearest_idx = i
			}
		}

		// 2. Steer towards sample point with bounded step size
		nearest_node := planner.nodes[nearest_idx]
		step_vec := sample_pt - nearest_node.position
		step_dist := core.vec3_length(step_vec)

		new_pos: core.Vec3
		if step_dist <= planner.step_size {
			new_pos = sample_pt
		} else {
			new_pos = nearest_node.position + step_vec * (planner.step_size / step_dist)
		}

		// 3. Collision check new segment
		if !rrt_is_segment_free(esdf, nearest_node.position, new_pos, planner.safety_margin) {
			continue
		}

		edge_cost := core.vec3_distance(nearest_node.position, new_pos)
		new_node_cost := nearest_node.cost + edge_cost
		best_parent := nearest_idx

		// 4. Choose best parent in neighborhood
		search_rad_sq := planner.search_radius * planner.search_radius
		for i in 0 ..< len(planner.nodes) {
			if i == nearest_idx do continue
			node_pos := planner.nodes[i].position
			if core.vec3_distance_sq(node_pos, new_pos) <= search_rad_sq {
				alt_cost := planner.nodes[i].cost + core.vec3_distance(node_pos, new_pos)
				if alt_cost < new_node_cost {
					if rrt_is_segment_free(esdf, node_pos, new_pos, planner.safety_margin) {
						new_node_cost = alt_cost
						best_parent = i
					}
				}
			}
		}

		new_node_idx := len(planner.nodes)
		append(&planner.nodes, RrtNode{
			position = new_pos,
			cost     = new_node_cost,
			parent   = best_parent,
		})

		// 5. Rewire neighbors
		for i in 0 ..< new_node_idx {
			if i == best_parent do continue
			node_pos := planner.nodes[i].position
			if core.vec3_distance_sq(new_pos, node_pos) <= search_rad_sq {
				rewire_cost := new_node_cost + core.vec3_distance(new_pos, node_pos)
				if rewire_cost < planner.nodes[i].cost {
					if rrt_is_segment_free(esdf, new_pos, node_pos, planner.safety_margin) {
						planner.nodes[i].cost = rewire_cost
						planner.nodes[i].parent = new_node_idx
					}
				}
			}
		}

		// 6. Check goal connection
		dist_to_goal := core.vec3_distance(new_pos, goal)
		if dist_to_goal <= planner.step_size {
			if rrt_is_segment_free(esdf, new_pos, goal, planner.safety_margin) {
				total_cost := new_node_cost + dist_to_goal
				if total_cost < best_goal_cost {
					best_goal_cost = total_cost
					best_goal_node = new_node_idx
				}
			}
		}
	}

	if best_goal_node == -1 {
		return false
	}

	// 7. Backtrack path from goal to start
	append(path_out, goal)
	curr := best_goal_node
	for curr != -1 {
		append(path_out, planner.nodes[curr].position)
		curr = planner.nodes[curr].parent
	}

	// Reverse path so it runs start -> goal
	n := len(path_out)
	for i in 0 ..< (n / 2) {
		tmp := path_out[i]
		path_out[i] = path_out[n - 1 - i]
		path_out[n - 1 - i] = tmp
	}

	return true
}
