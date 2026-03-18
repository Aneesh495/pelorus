package pelorus_core

import "core:math"

//
// Skew-symmetric 3x3 cross-product matrix operator (hat operator) for vector w in R^3.
// Maps w = [w1, w2, w3]^T to [w]_x in so(3).
//
so3_hat :: proc "contextless" (w: Vec3) -> Mat3 {
	return Mat3{
		 0.0, -w.z,  w.y,
		 w.z,  0.0, -w.x,
		-w.y,  w.x,  0.0,
	}
}

//
// Vee operator: extracts vector w in R^3 from skew-symmetric matrix Omega in so(3).
//
so3_vee :: proc "contextless" (m: Mat3) -> Vec3 {
	return Vec3{
		0.5 * (m[2, 1] - m[1, 2]),
		0.5 * (m[0, 2] - m[2, 0]),
		0.5 * (m[1, 0] - m[0, 1]),
	}
}

//
// Rodrigues exponential map: computes rotation matrix R in SO(3) from tangent vector phi in so(3).
// Applies Taylor expansion when theta < 1e-4 to prevent numerical instability.
//
so3_exp_mat3 :: proc "contextless" (phi: Vec3) -> Mat3 {
	theta_sq := vec3_length_sq(phi)
	theta := math.sqrt(theta_sq)
	hat := so3_hat(phi)
	hat2 := hat * hat

	a: f32
	b: f32

	if theta < 1e-4 {
		// Taylor expansion for sin(theta)/theta and (1 - cos(theta))/theta^2
		a = 1.0 - (theta_sq / 6.0) + (theta_sq * theta_sq / 120.0)
		b = 0.5 - (theta_sq / 24.0) + (theta_sq * theta_sq / 720.0)
	} else {
		a = math.sin(theta) / theta
		b = (1.0 - math.cos(theta)) / theta_sq
	}

	id := mat3_identity()
	res := id
	for row in 0 ..< 3 {
		for col in 0 ..< 3 {
			res[row, col] = id[row, col] + a * hat[row, col] + b * hat2[row, col]
		}
	}
	return res
}

//
// Exponential map directly yielding a unit Quaternion from tangent vector phi in so(3).
//
so3_exp_quat :: proc "contextless" (phi: Vec3) -> Quat {
	theta_sq := vec3_length_sq(phi)
	theta := math.sqrt(theta_sq)
	half_theta := theta * 0.5

	if theta < 1e-5 {
		c := 1.0 - (half_theta * half_theta * 0.5)
		s := 0.5 - (half_theta * half_theta / 12.0)
		return quat_normalize(Quat{w = c, x = phi.x * s, y = phi.y * s, z = phi.z * s})
	}

	c := math.cos(half_theta)
	s := math.sin(half_theta) / theta
	return Quat{
		w = c,
		x = phi.x * s,
		y = phi.y * s,
		z = phi.z * s,
	}
}

//
// Logarithmic map: computes tangent vector phi in so(3) from rotation matrix R in SO(3).
//
so3_log_mat3 :: proc "contextless" (r: Mat3) -> Vec3 {
	trace := r[0, 0] + r[1, 1] + r[2, 2]
	cos_theta := 0.5 * (trace - 1.0)
	cos_theta = math.clamp(cos_theta, -1.0, 1.0)
	theta := math.acos(cos_theta)

	if theta < 1e-5 {
		// Small angle limit: (1 / 2) * vee(R - R^T)
		return so3_vee(r)
	}

	scale := theta / (2.0 * math.sin(theta))
	v := so3_vee(r)
	return Vec3{v.x * scale, v.y * scale, v.z * scale}
}

//
// Logarithmic map: computes tangent vector phi in so(3) from unit Quaternion.
//
so3_log_quat :: proc "contextless" (q: Quat) -> Vec3 {
	vec_part := Vec3{q.x, q.y, q.z}
	vec_len := vec3_length(vec_part)

	if vec_len < 1e-6 {
		return Vec3{2.0 * q.x, 2.0 * q.y, 2.0 * q.z}
	}

	w_clamped := math.clamp(q.w, -1.0, 1.0)
	half_theta := math.atan2(vec_len, w_clamped)
	scale := 2.0 * half_theta / vec_len
	return Vec3{vec_part.x * scale, vec_part.y * scale, vec_part.z * scale}
}

//
// Left Jacobian of SO(3): J_l(phi).
// Maps perturbations in tangent space to variations in the Lie algebra.
//
so3_left_jacobian :: proc "contextless" (phi: Vec3) -> Mat3 {
	theta_sq := vec3_length_sq(phi)
	theta := math.sqrt(theta_sq)
	hat := so3_hat(phi)
	hat2 := hat * hat

	a: f32
	b: f32

	if theta < 1e-4 {
		a = 0.5 - (theta_sq / 24.0)
		b = (1.0 / 6.0) - (theta_sq / 120.0)
	} else {
		a = (1.0 - math.cos(theta)) / theta_sq
		b = (theta - math.sin(theta)) / (theta_sq * theta)
	}

	id := mat3_identity()
	res := id
	for r in 0 ..< 3 {
		for c in 0 ..< 3 {
			res[r, c] = id[r, c] + a * hat[r, c] + b * hat2[r, c]
		}
	}
	return res
}

//
// Inverse Left Jacobian of SO(3): J_l^{-1}(phi).
//
so3_left_jacobian_inv :: proc "contextless" (phi: Vec3) -> Mat3 {
	theta_sq := vec3_length_sq(phi)
	theta := math.sqrt(theta_sq)
	hat := so3_hat(phi)
	hat2 := hat * hat

	c: f32
	if theta < 1e-4 {
		c = (1.0 / 12.0) + (theta_sq / 720.0)
	} else {
		half_theta := 0.5 * theta
		cot_half := 1.0 / math.tan(half_theta)
		c = (1.0 / theta_sq) - (cot_half / (2.0 * theta))
	}

	id := mat3_identity()
	res := id
	for r in 0 ..< 3 {
		for col in 0 ..< 3 {
			res[r, col] = id[r, col] - 0.5 * hat[r, col] + c * hat2[r, col]
		}
	}
	return res
}

//
// Right Jacobian of SO(3): J_r(phi) = J_l(-phi).
//
so3_right_jacobian :: proc "contextless" (phi: Vec3) -> Mat3 {
	return so3_left_jacobian(-phi)
}

//
// Inverse Right Jacobian of SO(3): J_r^{-1}(phi) = J_l^{-1}(-phi).
//
so3_right_jacobian_inv :: proc "contextless" (phi: Vec3) -> Mat3 {
	return so3_left_jacobian_inv(-phi)
}
