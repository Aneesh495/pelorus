package pelorus_odometry

//
// Joseph-form covariance stabilization for ESKF to guarantee mathematical symmetry
// and positive semi-definiteness over extended operational horizons.
// Form: P = (I - K * H) * P * (I - K * H)^T + K * R * K^T.
//

//
// Enforce strict symmetry on 15x15 covariance matrix: P = 0.5 * (P + P^T).
//
eskf_cov_symmetrize :: proc "contextless" (cov: ^Mat15x15) {
	for r in 0 ..< 15 {
		for c in (r + 1) ..< 15 {
			avg := 0.5 * (cov[r][c] + cov[c][r])
			cov[r][c] = avg
			cov[c][r] = avg
		}
	}
}

//
// Clamp diagonal variances to minimum positive thresholds to prevent numerical overconfidence.
//
eskf_cov_clamp_diagonal :: proc "contextless" (cov: ^Mat15x15, min_var: f32 = 1e-7) {
	for i in 0 ..< 15 {
		if cov[i][i] < min_var {
			cov[i][i] = min_var
		}
	}
}

//
// Compute matrix trace of 15x15 covariance matrix: tr(P) = sum(P_{ii}).
//
eskf_cov_trace :: proc "contextless" (cov: Mat15x15) -> f32 {
	sum: f32 = 0.0
	for i in 0 ..< 15 {
		sum += cov[i][i]
	}
	return sum
}

//
// Full Joseph-form covariance update for 6-DOF measurement constraint.
// Ensures P remains positive-definite even under ill-conditioned measurement updates.
//
eskf_cov_joseph_update :: proc "contextless" (
	cov: ^Mat15x15,
	k_gain: [15][6]f32,
	meas_cov_pos: f32,
	meas_cov_rot: f32,
) {
	// 1. Build (I - K * H) matrix where H maps states [0..2] -> pos, [6..8] -> rot
	var_ikh: Mat15x15
	for r in 0 ..< 15 do var_ikh[r][r] = 1.0

	for r in 0 ..< 15 {
		// H col 0..2
		for c in 0 ..< 3 {
			var_ikh[r][c] -= k_gain[r][c]
		}
		// H col 6..8
		for c in 0 ..< 3 {
			var_ikh[r][c + 6] -= k_gain[r][c + 3]
		}
	}

	// 2. Term 1: Temp = (I - K * H) * P
	var_temp: Mat15x15
	for r in 0 ..< 15 {
		for c in 0 ..< 15 {
			sum: f32 = 0.0
			for k in 0 ..< 15 {
				sum += var_ikh[r][k] * cov[k][c]
			}
			var_temp[r][c] = sum
		}
	}

	// Term 1: P_term1 = Temp * (I - K * H)^T
	var_p1: Mat15x15
	for r in 0 ..< 15 {
		for c in 0 ..< 15 {
			sum: f32 = 0.0
			for k in 0 ..< 15 {
				sum += var_temp[r][k] * var_ikh[c][k]
			}
			var_p1[r][c] = sum
		}
	}

	// 3. Term 2: K * R * K^T
	// R is diagonal: [meas_cov_pos, meas_cov_pos, meas_cov_pos, meas_cov_rot, meas_cov_rot, meas_cov_rot]
	var_p2: Mat15x15
	for r in 0 ..< 15 {
		for c in 0 ..< 15 {
			sum: f32 = 0.0
			// Pos channels
			for k in 0 ..< 3 {
				sum += k_gain[r][k] * meas_cov_pos * k_gain[c][k]
			}
			// Rot channels
			for k in 3 ..< 6 {
				sum += k_gain[r][k] * meas_cov_rot * k_gain[c][k]
			}
			var_p2[r][c] = sum
		}
	}

	// 4. Combine: P_new = P_term1 + P_term2
	for r in 0 ..< 15 {
		for c in 0 ..< 15 {
			cov[r][c] = var_p1[r][c] + var_p2[r][c]
		}
	}

	// 5. Ensure symmetry and positive diagonal
	eskf_cov_symmetrize(cov)
	eskf_cov_clamp_diagonal(cov)
}
