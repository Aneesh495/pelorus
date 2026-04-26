package pelorus_sensor

import core "../core"

//
// First-order pre-integration Jacobians with respect to accelerometer and gyroscope biases.
// Enables instantaneous O(1) correction of pre-integrated delta states when biases update,
// avoiding expensive re-integration over thousands of IMU samples.
//
ImuJacobians :: struct {
	j_r_bg: core.Mat3, // d(delta_R) / d(b_g)
	j_v_ba: core.Mat3, // d(delta_v) / d(b_a)
	j_v_bg: core.Mat3, // d(delta_v) / d(b_g)
	j_p_ba: core.Mat3, // d(delta_p) / d(b_a)
	j_p_bg: core.Mat3, // d(delta_p) / d(b_g)
}

//
// Initialize Jacobians to zero matrices.
//
imu_jacobians_init :: proc(jac: ^ImuJacobians) {
	zero_mat := core.Mat3{
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
	}
	jac.j_r_bg = zero_mat
	jac.j_v_ba = zero_mat
	jac.j_v_bg = zero_mat
	jac.j_p_ba = zero_mat
	jac.j_p_bg = zero_mat
}

//
// Incremental propagation of Jacobians at each IMU integration step (Forster et al.).
//
imu_jacobians_step :: proc(
	jac: ^ImuJacobians,
	delta_r_mat: core.Mat3,
	unbiased_accel: core.Vec3,
	unbiased_gyro: core.Vec3,
	dt: f32,
) {
	dt_sq := dt * dt
	half_dt_sq := 0.5 * dt_sq

	step_rot_mat := core.so3_exp_mat3(unbiased_gyro * dt)
	step_rot_inv := core.Mat3{
		step_rot_mat[0, 0], step_rot_mat[1, 0], step_rot_mat[2, 0],
		step_rot_mat[0, 1], step_rot_mat[1, 1], step_rot_mat[2, 1],
		step_rot_mat[0, 2], step_rot_mat[1, 2], step_rot_mat[2, 2],
	}

	jr := core.so3_right_jacobian(unbiased_gyro * dt)
	accel_hat := core.so3_hat(unbiased_accel)

	// J_p_ba += J_v_ba * dt - 0.5 * delta_R * dt^2
	delta_r_dt_sq := core.mat3_scale(delta_r_mat, half_dt_sq)
	j_v_ba_dt := core.mat3_scale(jac.j_v_ba, dt)
	jac.j_p_ba = core.mat3_sub(core.mat3_add(jac.j_p_ba, j_v_ba_dt), delta_r_dt_sq)

	// J_p_bg += J_v_bg * dt - 0.5 * delta_R * [a]_x * J_r_bg * dt^2
	accel_term := (delta_r_mat * accel_hat) * jac.j_r_bg
	accel_term_scaled := core.mat3_scale(accel_term, half_dt_sq)
	j_v_bg_dt := core.mat3_scale(jac.j_v_bg, dt)
	jac.j_p_bg = core.mat3_sub(core.mat3_add(jac.j_p_bg, j_v_bg_dt), accel_term_scaled)

	// J_v_ba += -delta_R * dt
	delta_r_dt := core.mat3_scale(delta_r_mat, dt)
	jac.j_v_ba = core.mat3_sub(jac.j_v_ba, delta_r_dt)

	// J_v_bg += -delta_R * [a]_x * J_r_bg * dt
	accel_term_dt := core.mat3_scale(accel_term, dt)
	jac.j_v_bg = core.mat3_sub(jac.j_v_bg, accel_term_dt)

	// J_r_bg = step_R^T * J_r_bg - J_r * dt
	jr_dt := core.mat3_scale(jr, dt)
	jac.j_r_bg = core.mat3_sub(step_rot_inv * jac.j_r_bg, jr_dt)
}

//
// Correct pre-integrated measurements for small bias deviations (delta_ba, delta_bg)
// using first-order Taylor expansion:
// delta_R_corrected = delta_R * exp(J_r_bg * delta_bg)
// delta_v_corrected = delta_v + J_v_ba * delta_ba + J_v_bg * delta_bg
// delta_p_corrected = delta_p + J_p_ba * delta_ba + J_p_bg * delta_bg
//
imu_bias_correct_measurements :: proc(
	preint: ImuPreintegrated,
	jac: ImuJacobians,
	delta_ba: core.Vec3,
	delta_bg: core.Vec3,
) -> (corrected_p: core.Vec3, corrected_v: core.Vec3, corrected_r: core.Quat) {
	// 1. Correct position
	p_corr_ba := core.Vec3{
		jac.j_p_ba[0, 0] * delta_ba.x + jac.j_p_ba[0, 1] * delta_ba.y + jac.j_p_ba[0, 2] * delta_ba.z,
		jac.j_p_ba[1, 0] * delta_ba.x + jac.j_p_ba[1, 1] * delta_ba.y + jac.j_p_ba[1, 2] * delta_ba.z,
		jac.j_p_ba[2, 0] * delta_ba.x + jac.j_p_ba[2, 1] * delta_ba.y + jac.j_p_ba[2, 2] * delta_ba.z,
	}
	p_corr_bg := core.Vec3{
		jac.j_p_bg[0, 0] * delta_bg.x + jac.j_p_bg[0, 1] * delta_bg.y + jac.j_p_bg[0, 2] * delta_bg.z,
		jac.j_p_bg[1, 0] * delta_bg.x + jac.j_p_bg[1, 1] * delta_bg.y + jac.j_p_bg[1, 2] * delta_bg.z,
		jac.j_p_bg[2, 0] * delta_bg.x + jac.j_p_bg[2, 1] * delta_bg.y + jac.j_p_bg[2, 2] * delta_bg.z,
	}
	corrected_p = preint.delta_p + p_corr_ba + p_corr_bg

	// 2. Correct velocity
	v_corr_ba := core.Vec3{
		jac.j_v_ba[0, 0] * delta_ba.x + jac.j_v_ba[0, 1] * delta_ba.y + jac.j_v_ba[0, 2] * delta_ba.z,
		jac.j_v_ba[1, 0] * delta_ba.x + jac.j_v_ba[1, 1] * delta_ba.y + jac.j_v_ba[1, 2] * delta_ba.z,
		jac.j_v_ba[2, 0] * delta_ba.x + jac.j_v_ba[2, 1] * delta_ba.y + jac.j_v_ba[2, 2] * delta_ba.z,
	}
	v_corr_bg := core.Vec3{
		jac.j_v_bg[0, 0] * delta_bg.x + jac.j_v_bg[0, 1] * delta_bg.y + jac.j_v_bg[0, 2] * delta_bg.z,
		jac.j_v_bg[1, 0] * delta_bg.x + jac.j_v_bg[1, 1] * delta_bg.y + jac.j_v_bg[1, 2] * delta_bg.z,
		jac.j_v_bg[2, 0] * delta_bg.x + jac.j_v_bg[2, 1] * delta_bg.y + jac.j_v_bg[2, 2] * delta_bg.z,
	}
	corrected_v = preint.delta_v + v_corr_ba + v_corr_bg

	// 3. Correct rotation
	rot_corr_vector := core.Vec3{
		jac.j_r_bg[0, 0] * delta_bg.x + jac.j_r_bg[0, 1] * delta_bg.y + jac.j_r_bg[0, 2] * delta_bg.z,
		jac.j_r_bg[1, 0] * delta_bg.x + jac.j_r_bg[1, 1] * delta_bg.y + jac.j_r_bg[1, 2] * delta_bg.z,
		jac.j_r_bg[2, 0] * delta_bg.x + jac.j_r_bg[2, 1] * delta_bg.y + jac.j_r_bg[2, 2] * delta_bg.z,
	}
	rot_delta_step := core.so3_exp_quat(rot_corr_vector)
	corrected_r = core.quat_normalize(core.quat_mul(preint.delta_r, rot_delta_step))

	return
}
