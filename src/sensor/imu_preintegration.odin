package pelorus_sensor

import "core:math"
import core "../core"

//
// Pre-integrated IMU relative motion measurements between two keyframes (Forster et al.).
// Represents relative delta position, velocity, and rotation independent of world state.
//
ImuPreintegrated :: struct {
	delta_t:        f32,        // Accumulated duration between keyframes
	delta_p:        core.Vec3,  // Pre-integrated position delta
	delta_v:        core.Vec3,  // Pre-integrated velocity delta
	delta_r:        core.Quat,  // Pre-integrated rotation delta (quaternion form)
	delta_r_mat:    core.Mat3,  // Pre-integrated rotation delta (matrix form)
	linearized_ba:  core.Vec3,  // Accelerometer bias linearization point
	linearized_bg:  core.Vec3,  // Gyroscope bias linearization point
	noise_accel:    f32,        // Continuous accelerometer noise density (m/s^2 / sqrt(Hz))
	noise_gyro:     f32,        // Continuous gyroscope noise density (rad/s / sqrt(Hz))
	sample_count:   int,
}

//
// Initialize pre-integration structure with initial bias estimates and noise parameters.
//
imu_preintegrated_init :: proc(
	preint: ^ImuPreintegrated,
	ba: core.Vec3 = core.Vec3{0, 0, 0},
	bg: core.Vec3 = core.Vec3{0, 0, 0},
	noise_accel: f32 = 0.05,
	noise_gyro: f32 = 0.005,
) {
	preint.delta_t = 0.0
	preint.delta_p = core.vec3_zero()
	preint.delta_v = core.vec3_zero()
	preint.delta_r = core.quat_identity()
	preint.delta_r_mat = core.mat3_identity()
	preint.linearized_ba = ba
	preint.linearized_bg = bg
	preint.noise_accel = noise_accel
	preint.noise_gyro = noise_gyro
	preint.sample_count = 0
}

//
// Reset pre-integration for a new keyframe interval.
//
imu_preintegrated_reset :: proc(preint: ^ImuPreintegrated, ba, bg: core.Vec3) {
	preint.delta_t = 0.0
	preint.delta_p = core.vec3_zero()
	preint.delta_v = core.vec3_zero()
	preint.delta_r = core.quat_identity()
	preint.delta_r_mat = core.mat3_identity()
	preint.linearized_ba = ba
	preint.linearized_bg = bg
	preint.sample_count = 0
}

//
// Integrate a single IMU measurement (accel, gyro) over sample duration dt.
// Updates pre-integrated delta position, velocity, and rotation.
//
imu_preintegrated_integrate :: proc(
	preint: ^ImuPreintegrated,
	accel_raw: core.Vec3,
	gyro_raw: core.Vec3,
	dt: f32,
) {
	if dt <= 0.0 do return

	// Correct raw measurements with current linearization biases
	unbiased_accel := accel_raw - preint.linearized_ba
	unbiased_gyro := gyro_raw - preint.linearized_bg

	// Discrete rotation increment: exp((omega - b_g) * dt)
	rot_step := core.so3_exp_quat(unbiased_gyro * dt)
	rot_step_mat := core.so3_exp_mat3(unbiased_gyro * dt)

	// Acceleration rotated by accumulated rotation: delta_R_ik * (a_k - b_a)
	accel_rotated := core.quat_rotate_vec3(preint.delta_r, unbiased_accel)

	// Position update: delta_p += delta_v * dt + 0.5 * (delta_R * a) * dt^2
	dt_sq := dt * dt
	preint.delta_p += preint.delta_v * dt + accel_rotated * (0.5 * dt_sq)

	// Velocity update: delta_v += (delta_R * a) * dt
	preint.delta_v += accel_rotated * dt

	// Rotation update: delta_R = delta_R * rot_step
	preint.delta_r = core.quat_normalize(core.quat_mul(preint.delta_r, rot_step))
	preint.delta_r_mat = preint.delta_r_mat * rot_step_mat

	preint.delta_t += dt
	preint.sample_count += 1
}

//
// Predict navigation state at keyframe j given navigation state at keyframe i.
// Formulations:
// R_j = R_i * delta_R_ij
// v_j = v_i + g * delta_t + R_i * delta_v_ij
// p_j = p_i + v_i * delta_t + 0.5 * g * delta_t^2 + R_i * delta_p_ij
//
imu_predict_nav_state :: proc(
	preint: ^ImuPreintegrated,
	state_i: NavState,
) -> NavState {
	dt := preint.delta_t
	dt_sq := dt * dt
	g := state_i.gravity

	rot_i := state_i.pose.rotation
	pos_i := state_i.pose.translation
	vel_i := state_i.velocity

	// Predicted rotation
	rot_j := core.quat_normalize(core.quat_mul(rot_i, preint.delta_r))

	// Predicted velocity
	rot_delta_v := core.quat_rotate_vec3(rot_i, preint.delta_v)
	vel_j := vel_i + g * dt + rot_delta_v

	// Predicted position
	rot_delta_p := core.quat_rotate_vec3(rot_i, preint.delta_p)
	pos_j := pos_i + vel_i * dt + g * (0.5 * dt_sq) + rot_delta_p

	return NavState{
		timestamp  = state_i.timestamp + f64(dt),
		pose       = core.Transform3D{translation = pos_j, rotation = rot_j},
		velocity   = vel_j,
		accel_bias = preint.linearized_ba,
		gyro_bias  = preint.linearized_bg,
		gravity    = g,
	}
}
