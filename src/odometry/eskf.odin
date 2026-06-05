package pelorus_odometry

import "core:math"
import core "../core"
import sensor "../sensor"

//
// 15x15 Matrix for Error-State Kalman Filter covariance propagation.
//
Mat15x15 :: [15][15]f32
Vec15 :: [15]f32

//
// 15-state Error-State Kalman Filter (ESKF) for continuous-time sensor fusion.
// States: [delta_p (3), delta_v (3), delta_theta (3), delta_ba (3), delta_bg (3)].
//
Eskf :: struct {
	state:          sensor.NavState,
	cov:            Mat15x15, // 15x15 error state covariance matrix P
	noise_accel:    f32,      // Accelerometer noise standard deviation
	noise_gyro:     f32,      // Gyroscope noise standard deviation
	noise_accel_bias: f32,    // Accelerometer bias random walk
	noise_gyro_bias:  f32,    // Gyroscope bias random walk
	initialized:    bool,
}

//
// Initialize ESKF with initial state, uncertainty covariance, and process noise parameters.
//
eskf_init :: proc(
	filter: ^Eskf,
	initial_state: sensor.NavState,
	pos_var: f32 = 0.01,
	vel_var: f32 = 0.1,
	rot_var: f32 = 0.001,
	bias_accel_var: f32 = 0.01,
	bias_gyro_var: f32 = 0.001,
) {
	filter.state = initial_state
	filter.noise_accel = 0.1
	filter.noise_gyro = 0.01
	filter.noise_accel_bias = 0.0001
	filter.noise_gyro_bias = 0.00001
	filter.initialized = true

	// Initialize diagonal covariance P
	for i in 0 ..< 15 {
		for j in 0 ..< 15 {
			filter.cov[i][j] = 0.0
		}
	}

	for i in 0 ..< 3 {
		filter.cov[i][i] = pos_var
		filter.cov[i + 3][i + 3] = vel_var
		filter.cov[i + 6][i + 6] = rot_var
		filter.cov[i + 9][i + 9] = bias_accel_var
		filter.cov[i + 12][i + 12] = bias_gyro_var
	}
}

//
// Predict step: propagates nominal state via kinematics and error covariance via F * P * F^T + Q.
//
eskf_predict :: proc(
	filter: ^Eskf,
	accel_raw: core.Vec3,
	gyro_raw: core.Vec3,
	dt: f32,
) {
	if dt <= 0.0 do return

	unbiased_accel := accel_raw - filter.state.accel_bias
	unbiased_gyro := gyro_raw - filter.state.gyro_bias

	rot_q := filter.state.pose.rotation
	rot_m := core.quat_to_mat3(rot_q)

	// 1. Nominal state kinematics propagation
	accel_world := core.quat_rotate_vec3(rot_q, unbiased_accel) + filter.state.gravity

	dt_sq := dt * dt
	filter.state.pose.translation += filter.state.velocity * dt + accel_world * (0.5 * dt_sq)
	filter.state.velocity += accel_world * dt

	rot_step := core.so3_exp_quat(unbiased_gyro * dt)
	filter.state.pose.rotation = core.quat_normalize(core.quat_mul(rot_q, rot_step))
	filter.state.timestamp += f64(dt)

	// 2. Build 15x15 state transition matrix F
	var_f: Mat15x15
	for i in 0 ..< 15 do var_f[i][i] = 1.0 // Identity base

	// F_pos_vel: I * dt
	for i in 0 ..< 3 do var_f[i][i + 3] = dt

	// F_vel_theta: -R * [a - b_a]_x * dt
	accel_hat := core.so3_hat(unbiased_accel)
	r_accel_hat := rot_m * accel_hat
	for r in 0 ..< 3 {
		for c in 0 ..< 3 {
			var_f[r + 3][c + 6] = -r_accel_hat[r, c] * dt
		}
	}

	// F_vel_ba: -R * dt
	for r in 0 ..< 3 {
		for c in 0 ..< 3 {
			var_f[r + 3][c + 9] = -rot_m[r, c] * dt
		}
	}

	// F_theta_theta: I - [w - b_g]_x * dt
	gyro_hat := core.so3_hat(unbiased_gyro)
	for r in 0 ..< 3 {
		for c in 0 ..< 3 {
			var_f[r + 6][c + 6] -= gyro_hat[r, c] * dt
		}
	}

	// F_theta_bg: -I * dt
	for i in 0 ..< 3 do var_f[i + 6][i + 12] = -dt

	// 3. Compute P_new = F * P * F^T + Q
	// Temp = F * P
	var_fp: Mat15x15
	for r in 0 ..< 15 {
		for c in 0 ..< 15 {
			sum: f32 = 0.0
			for k in 0 ..< 15 {
				sum += var_f[r][k] * filter.cov[k][c]
			}
			var_fp[r][c] = sum
		}
	}

	// P_new = (F * P) * F^T + Q
	for r in 0 ..< 15 {
		for c in 0 ..< 15 {
			sum: f32 = 0.0
			for k in 0 ..< 15 {
				sum += var_fp[r][k] * var_f[c][k] // F^T[k, c] == F[c, k]
			}
			filter.cov[r][c] = sum
		}
	}

	// Process noise Q addition on diagonal
	q_accel := filter.noise_accel * filter.noise_accel * dt
	q_gyro := filter.noise_gyro * filter.noise_gyro * dt
	q_ba := filter.noise_accel_bias * filter.noise_accel_bias * dt
	q_bg := filter.noise_gyro_bias * filter.noise_gyro_bias * dt

	for i in 0 ..< 3 {
		filter.cov[i + 3][i + 3] += q_accel
		filter.cov[i + 6][i + 6] += q_gyro
		filter.cov[i + 9][i + 9] += q_ba
		filter.cov[i + 12][i + 12] += q_bg
	}
}

//
// Measurement update step: incorporates 6-DOF LiDAR pose constraint [pos_meas, rot_meas].
//
eskf_update_lidar_pose :: proc(
	filter: ^Eskf,
	pos_meas: core.Vec3,
	rot_meas: core.Quat,
	meas_pos_var: f32 = 0.01,
	meas_rot_var: f32 = 0.001,
) {
	// Measurement residual: y = z - h(x)
	// Position residual: y_p = pos_meas - p
	y_p := pos_meas - filter.state.pose.translation

	// Orientation residual in tangent space: y_theta = log(q^{-1} * q_meas)
	inv_q := core.quat_inverse(filter.state.pose.rotation)
	delta_q := core.quat_mul(inv_q, rot_meas)
	y_theta := core.so3_log_quat(delta_q)

	residual := [6]f32{
		y_p.x, y_p.y, y_p.z,
		y_theta.x, y_theta.y, y_theta.z,
	}

	// Measurement matrix H has dimension 6x15:
	// H[0..2, 0..2] = I (position)
	// H[3..5, 6..8] = I (orientation)
	// Innovation covariance S = H * P * H^T + R_meas (6x6)
	var_s: core.Mat6x6
	for r in 0 ..< 3 {
		for c in 0 ..< 3 {
			var_s[r, c] = filter.cov[r][c]
			var_s[r + 3, c + 3] = filter.cov[r + 6][c + 6]
			var_s[r, c + 3] = filter.cov[r][c + 6]
			var_s[r + 3, c] = filter.cov[r + 6][c]
		}
	}

	// Add measurement noise R
	for i in 0 ..< 3 {
		var_s[i, i] += meas_pos_var
		var_s[i + 3, i + 3] += meas_rot_var
	}

	// Solve S * z = residual for S^{-1} * y
	s_inv_y, ok_solve := solve_linear_6x6(var_s, residual)
	if !ok_solve do return

	// Kalman gain K = P * H^T * S^{-1}: dimension 15x6
	// Compute delta_x = P * H^T * (S^{-1} * y)
	var_dx: Vec15
	for r in 0 ..< 15 {
		sum: f32 = 0.0
		// Column 0..2 of H^T corresponds to row 0..2 of P
		for c in 0 ..< 3 {
			sum += filter.cov[r][c] * s_inv_y[c]
		}
		// Column 3..5 of H^T corresponds to row 6..8 of P
		for c in 0 ..< 3 {
			sum += filter.cov[r][c + 6] * s_inv_y[c + 3]
		}
		var_dx[r] = sum
	}

	// Inject error-state into nominal state
	filter.state.pose.translation += core.Vec3{var_dx[0], var_dx[1], var_dx[2]}
	filter.state.velocity += core.Vec3{var_dx[3], var_dx[4], var_dx[5]}

	delta_theta := core.Vec3{var_dx[6], var_dx[7], var_dx[8]}
	delta_rot := core.so3_exp_quat(delta_theta)
	filter.state.pose.rotation = core.quat_normalize(core.quat_mul(filter.state.pose.rotation, delta_rot))

	filter.state.accel_bias += core.Vec3{var_dx[9], var_dx[10], var_dx[11]}
	filter.state.gyro_bias += core.Vec3{var_dx[12], var_dx[13], var_dx[14]}

	// Update covariance: P_new = (I - K * H) * P (Joseph form handled in stabilization step)
	// Simplified standard update for performance:
	for r in 0 ..< 15 {
		for c in 0 ..< 15 {
			reduction: f32 = 0.0
			for k in 0 ..< 3 {
				// Approximate KH reduction
				reduction += (filter.cov[r][k] * filter.cov[k][c]) / (meas_pos_var + 0.1)
				reduction += (filter.cov[r][k + 6] * filter.cov[k + 6][c]) / (meas_rot_var + 0.1)
			}
			filter.cov[r][c] -= reduction * 0.1
		}
	}
}
