package pelorus_odometry

import "core:math"
import core "../core"
import sensor "../sensor"

//
// 3D Gaussian probability density cell for Normal Distributions Transform (NDT).
//
NdtCell :: struct {
	key:         u64,
	mean:        core.Vec3,
	inv_cov:     core.Mat3,
	point_count: int,
	occupied:    bool,
}

//
// 3D Normal Distributions Transform (NDT) Registration Engine.
// Models target scan as piecewise continuous probability distribution cells,
// optimizing transform likelihood via analytic Newton-Raphson iteration.
//
NdtSolver :: struct {
	voxel_size:  f32,
	inv_voxel:   f32,
	params:      RegistrationParams,
	cells:       []NdtCell,
	table_mask:  u64,
	d1:          f32,
	d2:          f32,
	owns_memory: bool,
}

//
// Initialize NdtSolver with voxel resolution and capacity.
//
ndt_solver_init :: proc(
	solver: ^NdtSolver,
	params: RegistrationParams,
	voxel_size: f32 = 1.0,
	capacity: int = 16384,
) {
	solver.params = params
	solver.voxel_size = voxel_size > 0.1 ? voxel_size : 1.0
	solver.inv_voxel = 1.0 / solver.voxel_size

	table_cap := 1024
	for table_cap < capacity * 2 {
		table_cap *= 2
	}

	solver.cells = make([]NdtCell, table_cap)
	solver.table_mask = u64(table_cap - 1)
	solver.d1 = 0.1
	solver.d2 = 1.0
	solver.owns_memory = true
}

//
// Release memory.
//
ndt_solver_destroy :: proc(solver: ^NdtSolver) {
	if solver.owns_memory && solver.cells != nil {
		delete(solver.cells)
	}
	solver.cells = nil
	solver.table_mask = 0
}

//
// Internal structure to accumulate points into cells before computing covariances.
//
CellAccumulator :: struct {
	key:          u64,
	sum_x:        f64,
	sum_y:        f64,
	sum_z:        f64,
	sum_xx:       f64,
	sum_yy:       f64,
	sum_zz:       f64,
	sum_xy:       f64,
	sum_xz:       f64,
	sum_yz:       f64,
	count:        int,
	occupied:     bool,
}

//
// Build 3D NDT Gaussian probability density representation from target PointCloud.
//
ndt_set_target :: proc(solver: ^NdtSolver, target_cloud: ^sensor.PointCloud) {
	n := len(target_cloud.points)
	if n == 0 do return

	table_cap := len(solver.cells)
	acc_table := make([]CellAccumulator, table_cap)
	defer delete(acc_table)

	mask := solver.table_mask
	inv := solver.inv_voxel

	// 1. Accumulate points into spatial bins
	for pt in target_cloud.points {
		p := pt.position
		gx := i32(math.floor(p.x * inv))
		gy := i32(math.floor(p.y * inv))
		gz := i32(math.floor(p.z * inv))

		key := core.voxel_hash_coords(gx, gy, gz)
		slot := key & mask

		for {
			acc := &acc_table[slot]
			if !acc.occupied {
				acc.key = key
				acc.sum_x = f64(p.x)
				acc.sum_y = f64(p.y)
				acc.sum_z = f64(p.z)
				acc.sum_xx = f64(p.x * p.x)
				acc.sum_yy = f64(p.y * p.y)
				acc.sum_zz = f64(p.z * p.z)
				acc.sum_xy = f64(p.x * p.y)
				acc.sum_xz = f64(p.x * p.z)
				acc.sum_yz = f64(p.y * p.z)
				acc.count = 1
				acc.occupied = true
				break
			}
			if acc.key == key {
				acc.sum_x += f64(p.x)
				acc.sum_y += f64(p.y)
				acc.sum_z += f64(p.z)
				acc.sum_xx += f64(p.x * p.x)
				acc.sum_yy += f64(p.y * p.y)
				acc.sum_zz += f64(p.z * p.z)
				acc.sum_xy += f64(p.x * p.y)
				acc.sum_xz += f64(p.x * p.z)
				acc.sum_yz += f64(p.y * p.z)
				acc.count += 1
				break
			}
			slot = (slot + 1) & mask
		}
	}

	// 2. Compute mean and inverted covariance for cells with sufficient points
	for i in 0 ..< len(solver.cells) {
		solver.cells[i].occupied = false
	}

	for i in 0 ..< table_cap {
		acc := acc_table[i]
		if !acc.occupied || acc.count < 5 do continue

		inv_n := 1.0 / f64(acc.count)
		mean_x := f32(acc.sum_x * inv_n)
		mean_y := f32(acc.sum_y * inv_n)
		mean_z := f32(acc.sum_z * inv_n)
		mean_vec := core.Vec3{mean_x, mean_y, mean_z}

		scale := 1.0 / f64(acc.count - 1)
		var_xx := f32((acc.sum_xx - acc.sum_x * acc.sum_x * inv_n) * scale)
		var_yy := f32((acc.sum_yy - acc.sum_y * acc.sum_y * inv_n) * scale)
		var_zz := f32((acc.sum_zz - acc.sum_z * acc.sum_z * inv_n) * scale)
		var_xy := f32((acc.sum_xy - acc.sum_x * acc.sum_y * inv_n) * scale)
		var_xz := f32((acc.sum_xz - acc.sum_x * acc.sum_z * inv_n) * scale)
		var_yz := f32((acc.sum_yz - acc.sum_y * acc.sum_z * inv_n) * scale)

		// Regularize covariance diagonal to prevent singularity
		reg := f32(0.01)
		cov := core.Mat3{
			var_xx + reg, var_xy,       var_xz,
			var_xy,       var_yy + reg, var_yz,
			var_xz,       var_yz,       var_zz + reg,
		}

		inv_cov, ok_inv := core.invert_cov_3x3(cov)
		if !ok_inv do continue

		slot := acc.key & mask
		for {
			if !solver.cells[slot].occupied {
				solver.cells[slot] = NdtCell{
					key         = acc.key,
					mean        = mean_vec,
					inv_cov     = inv_cov,
					point_count = acc.count,
					occupied    = true,
				}
				break
			}
			slot = (slot + 1) & mask
		}
	}
}

//
// Align source PointCloud to target NDT model using Newton optimization.
//
ndt_align :: proc(
	solver: ^NdtSolver,
	source_cloud: ^sensor.PointCloud,
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
	if n_src == 0 do return res

	current_tf := init_tf
	mask := solver.table_mask
	inv := solver.inv_voxel
	prev_score: f32 = 1e9

	for iter in 0 ..< solver.params.max_iterations {
		res.iterations = iter + 1

		var_h: core.Mat6x6
		var_g: [6]f32
		total_score: f32 = 0.0
		inliers := 0

		for pt in source_cloud.points {
			p_trans := core.transform3d_apply_point(current_tf, pt.position)

			gx := i32(math.floor(p_trans.x * inv))
			gy := i32(math.floor(p_trans.y * inv))
			gz := i32(math.floor(p_trans.z * inv))

			key := core.voxel_hash_coords(gx, gy, gz)
			slot := key & mask

			found_cell := false
			cell_idx := -1

			for {
				cell := &solver.cells[slot]
				if !cell.occupied do break
				if cell.key == key {
					found_cell = true
					cell_idx = int(slot)
					break
				}
				slot = (slot + 1) & mask
			}

			if !found_cell do continue

			cell := solver.cells[cell_idx]
			diff := p_trans - cell.mean

			// Mahalanobis distance term: diff^T * inv_cov * diff
			c_diff := core.Vec3{
				cell.inv_cov[0, 0] * diff.x + cell.inv_cov[0, 1] * diff.y + cell.inv_cov[0, 2] * diff.z,
				cell.inv_cov[1, 0] * diff.x + cell.inv_cov[1, 1] * diff.y + cell.inv_cov[1, 2] * diff.z,
				cell.inv_cov[2, 0] * diff.x + cell.inv_cov[2, 1] * diff.y + cell.inv_cov[2, 2] * diff.z,
			}
			d_sq := core.vec3_dot(diff, c_diff)
			if d_sq > 16.0 do continue // Chi-squared outlier gating

			e_term := math.exp(-0.5 * d_sq)
			total_score += -e_term
			inliers += 1

			// Jacobian of point w.r.t SE(3) pose: J_p = [ I, -[p]_x ]
			// Gradient of exponent: g_p = -inv_cov * diff
			gp := -c_diff * e_term
			cross_p := core.vec3_cross(p_trans, gp)

			j_grad := [6]f32{
				gp.x, gp.y, gp.z,
				cross_p.x, cross_p.y, cross_p.z,
			}

			// Approximate Gauss-Newton Hessian
			for r in 0 ..< 6 {
				for c in 0 ..< 6 {
					var_h[r, c] += j_grad[r] * j_grad[c]
				}
				var_g[r] += j_grad[r]
			}
		}

		if inliers < 6 do break

		res.inlier_count = inliers
		res.fitness_score = inliers > 0 ? (total_score / f32(inliers)) : 0.0
		res.information_matrix = var_h

		// Damping
		for d in 0 ..< 6 {
			var_h[d, d] += 1e-3
		}

		delta_xi_arr, ok_solve := solve_linear_6x6(var_h, var_g)
		if !ok_solve do break

		delta_xi := core.Twist6{
			rho = core.Vec3{delta_xi_arr[0], delta_xi_arr[1], delta_xi_arr[2]},
			phi = core.Vec3{delta_xi_arr[3], delta_xi_arr[4], delta_xi_arr[5]},
		}

		step_tf := core.se3_exp(delta_xi)
		current_tf = core.transform3d_mul(step_tf, current_tf)

		step_norm := core.vec3_length(delta_xi.rho) + core.vec3_length(delta_xi.phi)
		score_diff := math.abs(prev_score - total_score)
		prev_score = total_score

		if step_norm < solver.params.transformation_epsilon || score_diff < solver.params.euclidean_fitness_epsilon {
			res.converged = true
			break
		}
	}

	res.transform = current_tf
	res.inlier_ratio = n_src > 0 ? (f32(res.inlier_count) / f32(n_src)) : 0.0
	return res
}
