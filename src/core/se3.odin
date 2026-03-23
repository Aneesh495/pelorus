package pelorus_core

import "core:math"

//
// 6-DOF tangent vector (twist) in se(3): [rho (translation), phi (rotation)].
//
Twist6 :: struct {
	rho: Vec3, // Translational velocity / displacement
	phi: Vec3, // Rotational angular velocity / axis-angle
}

//
// 6x6 Adjoint matrix representation for SE(3).
//
Mat6x6 :: matrix[6, 6]f32

//
// Construct a Twist6 from linear and angular 3-vectors.
//
twist6_make :: proc "contextless" (rho, phi: Vec3) -> Twist6 {
	return Twist6{rho = rho, phi = phi}
}

//
// Zero twist.
//
twist6_zero :: proc "contextless" () -> Twist6 {
	return Twist6{rho = vec3_zero(), phi = vec3_zero()}
}

//
// 4x4 matrix wedge operator: maps 6-vector xi = [rho, phi]^T to se(3) Lie algebra element.
//
se3_hat :: proc "contextless" (xi: Twist6) -> Mat4 {
	phi_hat := so3_hat(xi.phi)
	return Mat4{
		phi_hat[0, 0], phi_hat[0, 1], phi_hat[0, 2], xi.rho.x,
		phi_hat[1, 0], phi_hat[1, 1], phi_hat[1, 2], xi.rho.y,
		phi_hat[2, 0], phi_hat[2, 1], phi_hat[2, 2], xi.rho.z,
		0.0,           0.0,           0.0,           0.0,
	}
}

//
// 6-vector vee operator: extracts Twist6 from 4x4 se(3) matrix.
//
se3_vee :: proc "contextless" (m: Mat4) -> Twist6 {
	so3_part := Mat3{
		m[0, 0], m[0, 1], m[0, 2],
		m[1, 0], m[1, 1], m[1, 2],
		m[2, 0], m[2, 1], m[2, 2],
	}
	return Twist6{
		rho = Vec3{m[0, 3], m[1, 3], m[2, 3]},
		phi = so3_vee(so3_part),
	}
}

//
// Exponential map: maps 6-DOF tangent vector xi in se(3) to rigid body Transform3D in SE(3).
// In SE(3), translation under exp is t = J_l(phi) * rho.
//
se3_exp :: proc "contextless" (xi: Twist6) -> Transform3D {
	rot_quat := so3_exp_quat(xi.phi)
	jl := so3_left_jacobian(xi.phi)

	// Translation: t = J_l(phi) * rho
	trans := Vec3{
		jl[0, 0] * xi.rho.x + jl[0, 1] * xi.rho.y + jl[0, 2] * xi.rho.z,
		jl[1, 0] * xi.rho.x + jl[1, 1] * xi.rho.y + jl[1, 2] * xi.rho.z,
		jl[2, 0] * xi.rho.x + jl[2, 1] * xi.rho.y + jl[2, 2] * xi.rho.z,
	}

	return Transform3D{
		translation = trans,
		rotation    = rot_quat,
	}
}

//
// Logarithmic map: maps rigid body Transform3D in SE(3) to 6-DOF tangent vector in se(3).
//
se3_log :: proc "contextless" (tf: Transform3D) -> Twist6 {
	phi := so3_log_quat(tf.rotation)
	jl_inv := so3_left_jacobian_inv(phi)

	// Recover rho: rho = J_l^{-1}(phi) * t
	rho := Vec3{
		jl_inv[0, 0] * tf.translation.x + jl_inv[0, 1] * tf.translation.y + jl_inv[0, 2] * tf.translation.z,
		jl_inv[1, 0] * tf.translation.x + jl_inv[1, 1] * tf.translation.y + jl_inv[1, 2] * tf.translation.z,
		jl_inv[2, 0] * tf.translation.x + jl_inv[2, 1] * tf.translation.y + jl_inv[2, 2] * tf.translation.z,
	}

	return Twist6{
		rho = rho,
		phi = phi,
	}
}

//
// Adjoint matrix representation Ad_T in R^{6x6} of rigid transformation T in SE(3).
// Ad_T = [ R,  [t]_x * R ]
//        [ 0,      R     ]
//
se3_adjoint :: proc "contextless" (tf: Transform3D) -> Mat6x6 {
	r := quat_to_mat3(tf.rotation)
	t_hat := so3_hat(tf.translation)
	t_hat_r := t_hat * r

	var_adj: Mat6x6
	// Block (0..2, 0..2) = R
	for i in 0 ..< 3 {
		for j in 0 ..< 3 {
			var_adj[i, j] = r[i, j]
		}
	}
	// Block (0..2, 3..5) = [t]_x * R
	for i in 0 ..< 3 {
		for j in 0 ..< 3 {
			var_adj[i, j + 3] = t_hat_r[i, j]
		}
	}
	// Block (3..5, 0..2) = 0
	for i in 0 ..< 3 {
		for j in 0 ..< 3 {
			var_adj[i + 3, j] = 0.0
		}
	}
	// Block (3..5, 3..5) = R
	for i in 0 ..< 3 {
		for j in 0 ..< 3 {
			var_adj[i + 3, j + 3] = r[i, j]
		}
	}

	return var_adj
}

//
// Apply 6x6 Adjoint transformation to 6-DOF Twist: xi_b = Ad_T * xi_a.
//
se3_adjoint_apply :: proc "contextless" (tf: Transform3D, xi: Twist6) -> Twist6 {
	r := quat_to_mat3(tf.rotation)
	t_hat := so3_hat(tf.translation)

	// Rotated rho: R * rho
	r_rho := Vec3{
		r[0, 0] * xi.rho.x + r[0, 1] * xi.rho.y + r[0, 2] * xi.rho.z,
		r[1, 0] * xi.rho.x + r[1, 1] * xi.rho.y + r[1, 2] * xi.rho.z,
		r[2, 0] * xi.rho.x + r[2, 1] * xi.rho.y + r[2, 2] * xi.rho.z,
	}

	// Rotated phi: R * phi
	r_phi := Vec3{
		r[0, 0] * xi.phi.x + r[0, 1] * xi.phi.y + r[0, 2] * xi.phi.z,
		r[1, 0] * xi.phi.x + r[1, 1] * xi.phi.y + r[1, 2] * xi.phi.z,
		r[2, 0] * xi.phi.x + r[2, 1] * xi.phi.y + r[2, 2] * xi.phi.z,
	}

	// Cross product term: [t]_x * (R * phi) = t x (R * phi)
	t_cross_r_phi := vec3_cross(tf.translation, r_phi)

	return Twist6{
		rho = r_rho + t_cross_r_phi,
		phi = r_phi,
	}
}

//
// Small adjoint ad_xi in R^{6x6} for Lie algebra element xi = [rho, phi]^T in se(3).
// ad_xi = [ [phi]_x,  [rho]_x ]
//         [    0,     [phi]_x ]
//
se3_ad_small :: proc "contextless" (xi: Twist6) -> Mat6x6 {
	phi_hat := so3_hat(xi.phi)
	rho_hat := so3_hat(xi.rho)

	var_ad: Mat6x6
	for i in 0 ..< 3 {
		for j in 0 ..< 3 {
			var_ad[i, j] = phi_hat[i, j]
			var_ad[i, j + 3] = rho_hat[i, j]
			var_ad[i + 3, j] = 0.0
			var_ad[i + 3, j + 3] = phi_hat[i, j]
		}
	}
	return var_ad
}
