package pelorus_odometry

import "core:math"
import core "../core"
import sensor "../sensor"

//
// Point-to-Plane and Point-to-Point Iterative Closest Point (ICP) Registration Engine.
// Solves non-linear least squares on SE(3) with M-estimator Huber loss for robust outlier rejection.
//
IcpSolver :: struct {
	params:           RegistrationParams,
	kdtree_cells:     []int,
	bin_mask:         int,
	entry_pool:       []core.SpatialNeighborCell,
	cell_size:        f32,
	owns_memory:      bool,
}

//
// Initialize ICP solver with parameters and working memory capacity.
//
icp_solver_init :: proc(solver: ^IcpSolver, params: RegistrationParams, capacity: int = 16384) {
	solver.params = params
	solver.cell_size = params.max_correspondence_dist > 0.1 ? params.max_correspondence_dist : 0.5

	table_cap := 1024
	for table_cap < capacity * 2 {
		table_cap *= 2
	}

	solver.kdtree_cells = make([]int, table_cap)
	solver.bin_mask = table_cap - 1
	solver.entry_pool = make([]core.SpatialNeighborCell, capacity)
	solver.owns_memory = true
}

//
// Release memory.
//
icp_solver_destroy :: proc(solver: ^IcpSolver) {
	if solver.owns_memory {
		if solver.kdtree_cells != nil do delete(solver.kdtree_cells)
		if solver.entry_pool != nil do delete(solver.entry_pool)
	}
	solver.kdtree_cells = nil
	solver.entry_pool = nil
	solver.bin_mask = 0
}

//
// Solve 6x6 symmetric positive definite linear system H * x = b via Gaussian elimination with partial pivoting.
//
solve_linear_6x6 :: proc "contextless" (h: core.Mat6x6, b: [6]f32) -> (x: [6]f32, ok: bool) {
	a := h
	rhs := b

	for i in 0 ..< 6 {
		// Pivot search
		max_val := math.abs(a[i, i])
		pivot_row := i
		for k in (i + 1) ..< 6 {
			if math.abs(a[k, i]) > max_val {
				max_val = math.abs(a[k, i])
				pivot_row = k
			}
		}

		if max_val < 1e-8 do return x, false

		// Swap rows
		if pivot_row != i {
			for c in 0 ..< 6 {
				tmp := a[i, c]
				a[i, c] = a[pivot_row, c]
				a[pivot_row, c] = tmp
			}
			tmp_rhs := rhs[i]
			rhs[i] = rhs[pivot_row]
			rhs[pivot_row] = tmp_rhs
		}

		// Eliminate
		inv_pivot := 1.0 / a[i, i]
		for k in (i + 1) ..< 6 {
			factor := a[k, i] * inv_pivot
			for c in i ..< 6 {
				a[k, c] -= factor * a[i, c]
			}
			rhs[k] -= factor * rhs[i]
		}
	}

	// Back substitution
	for i := 5; i >= 0; i -= 1 {
		sum: f32 = rhs[i]
		for c in (i + 1) ..< 6 {
			sum -= a[i, c] * x[c]
		}
		x[i] = sum / a[i, i]
	}

	return x, true
}

//
// Register source PointCloud against target PointCloud using Point-to-Plane ICP.
// Takes initial transform estimate and iteratively optimizes SE(3) pose.
//
icp_align :: proc(
	solver: ^IcpSolver,
	source_cloud: ^sensor.PointCloud,
	target_cloud: ^sensor.PointCloud,
	initial_guess: core.Transform3D = {},
) -> RegistrationResult {
	init_tf := initial_guess
	if init_tf.rotation.w == 0.0 && init_tf.rotation.x == 0.0 && init_tf.rotation.y == 0.0 && init_tf.rotation.z == 0.0 {
		init_tf = core.transform3d_identity()
	}

	res := RegistrationResult{
		transform     = init_tf,
		iterations    = 0,
		fitness_score = 0.0,
		inlier_count  = 0,
		converged     = false,
		is_degenerate = false,
	}

	n_src := len(source_cloud.points)
	n_tgt := len(target_cloud.points)
	if n_src == 0 || n_tgt == 0 do return res

	// 1. Build spatial index over target cloud
	for i in 0 ..< len(solver.kdtree_cells) {
		solver.kdtree_cells[i] = -1
	}

	inv_cell := 1.0 / solver.cell_size
	target_entries := math.min(n_tgt, len(solver.entry_pool))

	for i in 0 ..< target_entries {
		p := target_cloud.points[i].position
		gx := i32(math.floor(p.x * inv_cell))
		gy := i32(math.floor(p.y * inv_cell))
		gz := i32(math.floor(p.z * inv_cell))

		key := core.voxel_hash_coords(gx, gy, gz)
		bin := int(key) & solver.bin_mask

		solver.entry_pool[i] = core.SpatialNeighborCell{
			key         = key,
			point_index = i,
			next_in_bin = solver.kdtree_cells[bin],
			occupied    = true,
		}
		solver.kdtree_cells[bin] = i
	}

	current_tf := init_tf
	max_corr_dist_sq := solver.params.max_correspondence_dist * solver.params.max_correspondence_dist
	huber_delta := solver.params.huber_delta
	prev_error: f32 = 1e9

	// 2. Optimization Gauss-Newton loop
	for iter in 0 ..< solver.params.max_iterations {
		res.iterations = iter + 1

		var_h: core.Mat6x6
		var_b: [6]f32
		total_residual: f32 = 0.0
		inliers := 0

		// Accumulate linear system across correspondences
		for i in 0 ..< n_src {
			src_pt := source_cloud.points[i]
			// Transform source point into target frame: p_trans = T * p_src
			p_trans := core.transform3d_apply_point(current_tf, src_pt.position)

			gx := i32(math.floor(p_trans.x * inv_cell))
			gy := i32(math.floor(p_trans.y * inv_cell))
			gz := i32(math.floor(p_trans.z * inv_cell))

			// Nearest neighbor search in 3x3x3 target cells
			best_dist_sq := max_corr_dist_sq
			best_tgt_idx := -1

			for dx in -1 ..= 1 {
				for dy in -1 ..= 1 {
					for dz in -1 ..= 1 {
						n_key := core.voxel_hash_coords(gx + i32(dx), gy + i32(dy), gz + i32(dz))
						bin := int(n_key) & solver.bin_mask

						curr := solver.kdtree_cells[bin]
						for curr != -1 {
							cell := solver.entry_pool[curr]
							if cell.key == n_key {
								tgt_p := target_cloud.points[cell.point_index].position
								d_sq := core.vec3_distance_sq(p_trans, tgt_p)
								if d_sq < best_dist_sq {
									best_dist_sq = d_sq
									best_tgt_idx = cell.point_index
								}
							}
							curr = cell.next_in_bin
						}
					}
				}
			}

			if best_tgt_idx == -1 do continue

			tgt_pt := target_cloud.points[best_tgt_idx]
			normal := tgt_pt.normal
			if core.vec3_length_sq(normal) < 0.5 {
				// Fallback to point-to-point if normal absent
				normal = core.vec3_normalize(p_trans - tgt_pt.position)
			}

			// Point-to-plane residual: r = n^T * (p_trans - p_tgt)
			diff := p_trans - tgt_pt.position
			residual := core.vec3_dot(normal, diff)
			abs_res := math.abs(residual)

			// Huber robust weighting
			weight: f32 = 1.0
			if abs_res > huber_delta {
				weight = huber_delta / (abs_res + 1e-6)
			}

			// Jacobian row: J = [ n_x, n_y, n_z, (p_trans x n)_x, (p_trans x n)_y, (p_trans x n)_z ]
			cross_pn := core.vec3_cross(p_trans, normal)
			j_row := [6]f32{
				normal.x, normal.y, normal.z,
				cross_pn.x, cross_pn.y, cross_pn.z,
			}

			// Accumulate H += weight * J^T * J, b += -weight * J^T * r
			for r in 0 ..< 6 {
				for c in 0 ..< 6 {
					var_h[r, c] += weight * j_row[r] * j_row[c]
				}
				var_b[r] += -weight * j_row[r] * residual
			}

			total_residual += abs_res
			inliers += 1
		}

		if inliers < 6 {
			// Insufficient correspondences to constrain 6-DOF
			break
		}

		mean_error := total_residual / f32(inliers)
		res.inlier_count = inliers
		res.fitness_score = mean_error
		res.information_matrix = var_h

		// Levenberg-Marquardt damping: H + lambda * I
		lambda: f32 = 1e-4
		for d in 0 ..< 6 {
			var_h[d, d] += lambda
		}

		delta_xi_arr, ok_solve := solve_linear_6x6(var_h, var_b)
		if !ok_solve do break

		delta_xi := core.Twist6{
			rho = core.Vec3{delta_xi_arr[0], delta_xi_arr[1], delta_xi_arr[2]},
			phi = core.Vec3{delta_xi_arr[3], delta_xi_arr[4], delta_xi_arr[5]},
		}

		// Update transformation: T_{new} = exp(delta_xi) * T_{current}
		step_tf := core.se3_exp(delta_xi)
		current_tf = core.transform3d_mul(step_tf, current_tf)

		// Check convergence
		step_norm := core.vec3_length(delta_xi.rho) + core.vec3_length(delta_xi.phi)
		error_diff := math.abs(prev_error - mean_error)
		prev_error = mean_error

		if step_norm < solver.params.transformation_epsilon || error_diff < solver.params.euclidean_fitness_epsilon {
			res.converged = true
			break
		}
	}

	res.transform = current_tf
	res.inlier_ratio = n_src > 0 ? (f32(res.inlier_count) / f32(n_src)) : 0.0
	return res
}
