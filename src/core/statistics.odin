package pelorus_core

import "core:math"

//
// Chi-squared distribution critical value thresholds for statistical gating.
// Keyed by degrees of freedom: [1..9].
//
CHI2_GATE_95 :: [10]f32{
	0.0,
	3.841,  // 1 DOF
	5.991,  // 2 DOF
	7.815,  // 3 DOF
	9.488,  // 4 DOF
	11.070, // 5 DOF
	12.592, // 6 DOF
	14.067, // 7 DOF
	15.507, // 8 DOF
	16.919, // 9 DOF
}

CHI2_GATE_99 :: [10]f32{
	0.0,
	6.635,  // 1 DOF
	9.210,  // 2 DOF
	11.345, // 3 DOF
	13.277, // 4 DOF
	15.086, // 5 DOF
	16.812, // 6 DOF
	18.475, // 7 DOF
	20.090, // 8 DOF
	21.666, // 9 DOF
}

//
// Compute 3x3 Cholesky decomposition A = L * L^T.
// Returns lower triangular matrix L and success flag (false if matrix not positive definite).
//
cholesky_3x3 :: proc "contextless" (a: Mat3) -> (l: Mat3, ok: bool) {
	l = mat3_identity()

	// Row 0
	if a[0, 0] <= 0.0 do return l, false
	l[0, 0] = math.sqrt(a[0, 0])
	l[1, 0] = a[1, 0] / l[0, 0]
	l[2, 0] = a[2, 0] / l[0, 0]

	// Row 1
	d1 := a[1, 1] - l[1, 0] * l[1, 0]
	if d1 <= 0.0 do return l, false
	l[1, 1] = math.sqrt(d1)
	l[2, 1] = (a[2, 1] - l[2, 0] * l[1, 0]) / l[1, 1]

	// Row 2
	d2 := a[2, 2] - (l[2, 0] * l[2, 0] + l[2, 1] * l[2, 1])
	if d2 <= 0.0 do return l, false
	l[2, 2] = math.sqrt(d2)

	// Upper triangle is zero
	l[0, 1] = 0.0
	l[0, 2] = 0.0
	l[1, 2] = 0.0

	return l, true
}

//
// Invert a 3x3 symmetric positive-definite covariance matrix via Cholesky decomposition.
//
invert_cov_3x3 :: proc "contextless" (cov: Mat3) -> (inv: Mat3, ok: bool) {
	l, chol_ok := cholesky_3x3(cov)
	if !chol_ok do return mat3_identity(), false

	// Invert lower triangular matrix L: L_inv
	l_inv := mat3_identity()
	l_inv[0, 0] = 1.0 / l[0, 0]
	l_inv[1, 0] = -l[1, 0] * l_inv[0, 0] / l[1, 1]
	l_inv[1, 1] = 1.0 / l[1, 1]
	l_inv[2, 0] = -(l[2, 0] * l_inv[0, 0] + l[2, 1] * l_inv[1, 0]) / l[2, 2]
	l_inv[2, 1] = -l[2, 1] * l_inv[1, 1] / l[2, 2]
	l_inv[2, 2] = 1.0 / l[2, 2]

	// inv = L_inv^T * L_inv
	for r in 0 ..< 3 {
		for c in 0 ..< 3 {
			sum: f32 = 0.0
			for k in 0 ..< 3 {
				sum += l_inv[k, r] * l_inv[k, c]
			}
			inv[r, c] = sum
		}
	}

	return inv, true
}

//
// Compute squared Mahalanobis distance between observation x and mean mu:
// d_M^2 = (x - mu)^T * Sigma^{-1} * (x - mu).
//
mahalanobis_distance_sq_3d :: proc "contextless" (x, mu: Vec3, cov: Mat3) -> f32 {
	inv_cov, ok := invert_cov_3x3(cov)
	diff := x - mu
	if !ok {
		// Fallback to Euclidean squared distance if covariance is singular
		return vec3_length_sq(diff)
	}

	// v = inv_cov * diff
	vx := inv_cov[0, 0] * diff.x + inv_cov[0, 1] * diff.y + inv_cov[0, 2] * diff.z
	vy := inv_cov[1, 0] * diff.x + inv_cov[1, 1] * diff.y + inv_cov[1, 2] * diff.z
	vz := inv_cov[2, 0] * diff.x + inv_cov[2, 1] * diff.y + inv_cov[2, 2] * diff.z

	return diff.x * vx + diff.y * vy + diff.z * vz
}

//
// Compute sample mean and 3x3 covariance matrix over a point slice.
//
compute_mean_covariance_3d :: proc(points: []Vec3) -> (mean: Vec3, cov: Mat3) {
	n := len(points)
	if n == 0 {
		return vec3_zero(), mat3_identity()
	}

	// 1. Mean
	acc := vec3_zero()
	for p in points {
		acc += p
	}
	mean = acc * (1.0 / f32(n))

	if n < 2 {
		return mean, mat3_identity()
	}

	// 2. Covariance: sum((p - mean) * (p - mean)^T) / (n - 1)
	cov_acc: [3][3]f32
	for p in points {
		d := p - mean
		cov_acc[0][0] += d.x * d.x
		cov_acc[0][1] += d.x * d.y
		cov_acc[0][2] += d.x * d.z
		cov_acc[1][1] += d.y * d.y
		cov_acc[1][2] += d.y * d.z
		cov_acc[2][2] += d.z * d.z
	}

	cov_acc[1][0] = cov_acc[0][1]
	cov_acc[2][0] = cov_acc[0][2]
	cov_acc[2][1] = cov_acc[1][2]

	scale := 1.0 / f32(n - 1)
	for r in 0 ..< 3 {
		for c in 0 ..< 3 {
			cov[r, c] = cov_acc[r][c] * scale
		}
	}

	return mean, cov
}

//
// Jacobi eigenvalue algorithm for 3x3 real symmetric matrices.
// Computes eigenvalues (sorted descending) and orthogonal eigenvectors (columns of V).
// Used in Principal Component Analysis (PCA) for surface normal estimation and bounding box fitting.
//
eigen_decomposition_3x3 :: proc "contextless" (a: Mat3) -> (eigenvalues: Vec3, eigenvectors: Mat3) {
	v := mat3_identity()
	mat := a

	max_rotations :: 30
	for _ in 0 ..< max_rotations {
		// Find off-diagonal element of maximum magnitude
		p := 0
		q := 1
		max_val := math.abs(mat[0, 1])

		if math.abs(mat[0, 2]) > max_val {
			p = 0
			q = 2
			max_val = math.abs(mat[0, 2])
		}
		if math.abs(mat[1, 2]) > max_val {
			p = 1
			q = 2
			max_val = math.abs(mat[1, 2])
		}

		if max_val < 1e-7 {
			break
		}

		// Calculate Jacobi rotation angle
		app := mat[p, p]
		aqq := mat[q, q]
		apq := mat[p, q]

		theta := 0.5 * (aqq - app) / apq
		t := 1.0 / (math.abs(theta) + math.sqrt(1.0 + theta * theta))
		if theta < 0.0 do t = -t

		c := 1.0 / math.sqrt(1.0 + t * t)
		s := t * c
		tau := s / (1.0 + c)

		// Update matrix elements
		mat[p, p] -= t * apq
		mat[q, q] += t * apq
		mat[p, q] = 0.0
		mat[q, p] = 0.0

		for r in 0 ..< 3 {
			if r != p && r != q {
				arp := mat[r, p]
				arq := mat[r, q]
				mat[r, p] -= s * (arq + tau * arp)
				mat[p, r] = mat[r, p]
				mat[r, q] += s * (arp - tau * arq)
				mat[q, r] = mat[r, q]
			}
		}

		// Accumulate eigenvectors in V
		for r in 0 ..< 3 {
			vrp := v[r, p]
			vrq := v[r, q]
			v[r, p] -= s * (vrq + tau * vrp)
			v[r, q] += s * (vrp - tau * vrq)
		}
	}

	eigenvalues = Vec3{mat[0, 0], mat[1, 1], mat[2, 2]}
	eigenvectors = v

	// Sort eigenvalues and corresponding eigenvectors in descending order
	for i in 0 ..< 2 {
		for j in (i + 1) ..< 3 {
			val_i := (i == 0) ? eigenvalues.x : ((i == 1) ? eigenvalues.y : eigenvalues.z)
			val_j := (j == 0) ? eigenvalues.x : ((j == 1) ? eigenvalues.y : eigenvalues.z)

			if val_j > val_i {
				// Swap eigenvalues
				if i == 0 && j == 1 {
					tmp := eigenvalues.x; eigenvalues.x = eigenvalues.y; eigenvalues.y = tmp
				} else if i == 0 && j == 2 {
					tmp := eigenvalues.x; eigenvalues.x = eigenvalues.z; eigenvalues.z = tmp
				} else if i == 1 && j == 2 {
					tmp := eigenvalues.y; eigenvalues.y = eigenvalues.z; eigenvalues.z = tmp
				}

				// Swap eigenvector columns in V
				for r in 0 ..< 3 {
					tmp := eigenvectors[r, i]
					eigenvectors[r, i] = eigenvectors[r, j]
					eigenvectors[r, j] = tmp
				}
			}
		}
	}

	return eigenvalues, eigenvectors
}
