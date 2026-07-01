package pelorus_tracking

import "core:math"
import core "../core"

//
// 7-state Constant Turn Rate and Velocity (CTRV) kinematic state vector:
// [px, py, pz, v (speed), yaw, yaw_rate, vz].
//
CTRV_DIM :: 7
Mat7x7 :: [7][7]f32
Vec7 :: [7]f32

//
// 3D CTRV Extended Kalman Filter for dynamic obstacle tracking.
// Accurately predicts curved trajectories and vehicle turning behaviors.
//
KalmanFilter3D :: struct {
	state:             Vec7,    // [px, py, pz, v, yaw, yaw_rate, vz]
	cov:               Mat7x7,  // 7x7 state covariance matrix P
	noise_accel_horiz: f32,     // Horizontal linear acceleration process noise
	noise_yaw_accel:   f32,     // Yaw acceleration process noise
	noise_accel_vert:  f32,     // Vertical linear acceleration process noise
	dimensions:        core.Vec3, // (length, width, height) of tracked obstacle
}

//
// Initialize 3D CTRV Kalman filter at initial detection position and heading.
//
kalman3d_init :: proc(
	kf: ^KalmanFilter3D,
	pos: core.Vec3,
	yaw: f32,
	dimensions: core.Vec3,
	initial_speed: f32 = 0.0,
) {
	kf.state = Vec7{pos.x, pos.y, pos.z, initial_speed, yaw, 0.0, 0.0}
	kf.dimensions = dimensions
	kf.noise_accel_horiz = 1.0 // 1 m/s^2
	kf.noise_yaw_accel = 0.5   // 0.5 rad/s^2
	kf.noise_accel_vert = 0.5

	for r in 0 ..< CTRV_DIM {
		for c in 0 ..< CTRV_DIM {
			kf.cov[r][c] = 0.0
		}
	}

	// Initial diagonal uncertainty
	kf.cov[0][0] = 0.2 // Pos X var
	kf.cov[1][1] = 0.2 // Pos Y var
	kf.cov[2][2] = 0.2 // Pos Z var
	kf.cov[3][3] = 4.0 // Speed var
	kf.cov[4][4] = 0.1 // Yaw var
	kf.cov[5][5] = 0.2 // Yaw rate var
	kf.cov[6][6] = 1.0 // Vz var
}

//
// Normalize angle into [-PI, PI].
//
angle_normalize :: proc "contextless" (angle_in: f32) -> f32 {
	a := angle_in
	for a > f32(math.PI) do a -= f32(2.0 * math.PI)
	for a < -f32(math.PI) do a += f32(2.0 * math.PI)
	return a
}

//
// Predict step: propagates state non-linearly via CTRV kinematics and updates covariance.
//
kalman3d_predict :: proc(kf: ^KalmanFilter3D, dt: f32) {
	if dt <= 0.0 do return

	px := kf.state[0]
	py := kf.state[1]
	pz := kf.state[2]
	v := kf.state[3]
	yaw := kf.state[4]
	yaw_rate := kf.state[5]
	vz := kf.state[6]

	// 1. Non-linear state transition
	new_px: f32
	new_py: f32

	if math.abs(yaw_rate) > 1e-4 {
		// Curved path
		inv_yr := 1.0 / yaw_rate
		new_px = px + (v * inv_yr) * (math.sin(yaw + yaw_rate * dt) - math.sin(yaw))
		new_py = py + (v * inv_yr) * (-math.cos(yaw + yaw_rate * dt) + math.cos(yaw))
	} else {
		// Straight path
		new_px = px + v * math.cos(yaw) * dt
		new_py = py + v * math.sin(yaw) * dt
	}

	new_pz := pz + vz * dt
	new_v := v
	new_yaw := angle_normalize(yaw + yaw_rate * dt)
	new_yaw_rate := yaw_rate
	new_vz := vz

	kf.state[0] = new_px
	kf.state[1] = new_py
	kf.state[2] = new_pz
	kf.state[3] = new_v
	kf.state[4] = new_yaw
	kf.state[5] = new_yaw_rate
	kf.state[6] = new_vz

	// 2. Jacobian matrix F = df / dx
	var_f: Mat7x7
	for i in 0 ..< CTRV_DIM do var_f[i][i] = 1.0

	var_f[2][6] = dt // dpz / dvz

	if math.abs(yaw_rate) > 1e-4 {
		inv_yr := 1.0 / yaw_rate
		s1 := math.sin(yaw + yaw_rate * dt)
		s0 := math.sin(yaw)
		c1 := math.cos(yaw + yaw_rate * dt)
		c0 := math.cos(yaw)

		var_f[0][3] = inv_yr * (s1 - s0) // dpx / dv
		var_f[0][4] = (v * inv_yr) * (c1 - c0) // dpx / dyaw
		var_f[0][5] = (v * dt * inv_yr) * c1 - (v * inv_yr * inv_yr) * (s1 - s0) // dpx / dyaw_rate

		var_f[1][3] = inv_yr * (-c1 + c0) // dpy / dv
		var_f[1][4] = (v * inv_yr) * (s1 - s0) // dpy / dyaw
		var_f[1][5] = (v * dt * inv_yr) * s1 - (v * inv_yr * inv_yr) * (-c1 + c0) // dpy / dyaw_rate
	} else {
		var_f[0][3] = math.cos(yaw) * dt
		var_f[0][4] = -v * math.sin(yaw) * dt
		var_f[1][3] = math.sin(yaw) * dt
		var_f[1][4] = v * math.cos(yaw) * dt
	}

	var_f[4][5] = dt // dyaw / dyaw_rate

	// 3. Covariance propagation: P = F * P * F^T + Q
	var_fp: Mat7x7
	for r in 0 ..< CTRV_DIM {
		for c in 0 ..< CTRV_DIM {
			sum: f32 = 0.0
			for k in 0 ..< CTRV_DIM {
				sum += var_f[r][k] * kf.cov[k][c]
			}
			var_fp[r][c] = sum
		}
	}

	for r in 0 ..< CTRV_DIM {
		for c in 0 ..< CTRV_DIM {
			sum: f32 = 0.0
			for k in 0 ..< CTRV_DIM {
				sum += var_fp[r][k] * var_f[c][k]
			}
			kf.cov[r][c] = sum
		}
	}

	// Add process noise Q
	dt_sq := dt * dt
	q_a := kf.noise_accel_horiz * kf.noise_accel_horiz * dt_sq
	q_yaw := kf.noise_yaw_accel * kf.noise_yaw_accel * dt_sq
	q_vz := kf.noise_accel_vert * kf.noise_accel_vert * dt_sq

	kf.cov[0][0] += 0.25 * dt_sq * q_a
	kf.cov[1][1] += 0.25 * dt_sq * q_a
	kf.cov[2][2] += 0.25 * dt_sq * q_vz
	kf.cov[3][3] += q_a
	kf.cov[4][4] += 0.25 * dt_sq * q_yaw
	kf.cov[5][5] += q_yaw
	kf.cov[6][6] += q_vz
}

//
// Measurement update step from 3D bounding box detection: z = [meas_px, meas_py, meas_pz, meas_yaw].
//
kalman3d_update :: proc(
	kf: ^KalmanFilter3D,
	pos_meas: core.Vec3,
	yaw_meas: f32,
	dimensions: core.Vec3,
	pos_noise: f32 = 0.1,
	yaw_noise: f32 = 0.05,
) {
	// Measurement matrix H has dimension 4x7:
	// H[0, 0] = 1 (px), H[1, 1] = 1 (py), H[2, 2] = 1 (pz), H[3, 4] = 1 (yaw)
	y_px := pos_meas.x - kf.state[0]
	y_py := pos_meas.y - kf.state[1]
	y_pz := pos_meas.z - kf.state[2]
	y_yaw := angle_normalize(yaw_meas - kf.state[4])

	meas_res := [4]f32{y_px, y_py, y_pz, y_yaw}

	// Innovation covariance S (4x4)
	s00 := kf.cov[0][0] + pos_noise
	s11 := kf.cov[1][1] + pos_noise
	s22 := kf.cov[2][2] + pos_noise
	s33 := kf.cov[4][4] + yaw_noise

	// Diagonal inverse S^{-1}
	inv_s0 := 1.0 / s00
	inv_s1 := 1.0 / s11
	inv_s2 := 1.0 / s22
	inv_s3 := 1.0 / s33

	// Kalman gain K = P * H^T * S^{-1}: dimension 7x4
	for r in 0 ..< CTRV_DIM {
		k0 := kf.cov[r][0] * inv_s0
		k1 := kf.cov[r][1] * inv_s1
		k2 := kf.cov[r][2] * inv_s2
		k3 := kf.cov[r][4] * inv_s3

		dx := k0 * meas_res[0] + k1 * meas_res[1] + k2 * meas_res[2] + k3 * meas_res[3]
		kf.state[r] += dx
	}

	kf.state[4] = angle_normalize(kf.state[4])

	// Covariance update: P = (I - K * H) * P
	for r in 0 ..< CTRV_DIM {
		kf.cov[r][0] *= (1.0 - kf.cov[0][0] * inv_s0)
		kf.cov[r][1] *= (1.0 - kf.cov[1][1] * inv_s1)
		kf.cov[r][2] *= (1.0 - kf.cov[2][2] * inv_s2)
		kf.cov[r][4] *= (1.0 - kf.cov[4][4] * inv_s3)
	}

	// Smooth dimensions (exponential moving average)
	kf.dimensions = kf.dimensions * 0.8 + dimensions * 0.2
}

//
// Extract current estimated velocity vector in world Cartesian frame.
//
kalman3d_velocity_world :: proc "contextless" (kf: ^KalmanFilter3D) -> core.Vec3 {
	v := kf.state[3]
	yaw := kf.state[4]
	vz := kf.state[6]
	return core.Vec3{v * math.cos(yaw), v * math.sin(yaw), vz}
}
