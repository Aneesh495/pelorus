package pelorus_core

import "core:math"
import "core:math/linalg"

//
// Fundamental scalar precision types.
//
Float :: f32
Double :: f64

//
// Vector definitions with standard dimensionalities.
//
Vec2 :: [2]f32
Vec3 :: [3]f32
Vec4 :: [4]f32
Vec6 :: [6]f32

Vec2d :: [2]f64
Vec3d :: [3]f64
Vec4d :: [4]f64

//
// Matrix representations (column-major in memory, matching OpenGL and standard SIMD).
//
Mat2 :: matrix[2, 2]f32
Mat3 :: matrix[3, 3]f32
Mat4 :: matrix[4, 4]f32
Mat6 :: matrix[6, 6]f32

Mat3d :: matrix[3, 3]f64
Mat4d :: matrix[4, 4]f64

//
// Unit quaternion representation for 3D rotations: q = w + xi + yj + zk.
//
Quat :: struct {
	w: f32,
	x: f32,
	y: f32,
	z: f32,
}

//
// Dual quaternion representing simultaneous 3D rotation and translation.
//
DualQuat :: struct {
	real: Quat,
	dual: Quat,
}

//
// Rigid body spatial transformation in 3D Euclidean space SE(3).
//
Transform3D :: struct {
	translation: Vec3,
	rotation:    Quat,
}

//
// Standard reference frame identifiers in autonomous robotics.
//
FrameId :: enum u8 {
	World,      // Fixed inertial navigation frame (East-North-Up or local tangent)
	Odom,       // Odometry-consistent smooth continuous frame
	BaseLink,   // Mobile platform / vehicle center of mass coordinate frame
	SensorLidar,// Primary LiDAR optical center
	SensorImu,  // Inertial measurement unit accelerometer/gyroscope origin
	SensorCam,  // Camera optical center (Z forward, X right, Y down)
}

//
// Vector construction and utility routines.
//

vec2_zero :: proc "contextless" () -> Vec2 {
	return Vec2{0, 0}
}

vec3_zero :: proc "contextless" () -> Vec3 {
	return Vec3{0, 0, 0}
}

vec4_zero :: proc "contextless" () -> Vec4 {
	return Vec4{0, 0, 0, 0}
}

vec3_dot :: proc "contextless" (a, b: Vec3) -> f32 {
	return a.x * b.x + a.y * b.y + a.z * b.z
}

vec3_cross :: proc "contextless" (a, b: Vec3) -> Vec3 {
	return Vec3{
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x,
	}
}

vec3_length_sq :: proc "contextless" (v: Vec3) -> f32 {
	return vec3_dot(v, v)
}

vec3_length :: proc "contextless" (v: Vec3) -> f32 {
	return math.sqrt(vec3_length_sq(v))
}

vec3_normalize :: proc "contextless" (v: Vec3) -> Vec3 {
	len := vec3_length(v)
	if len < 1e-8 {
		return vec3_zero()
	}
	inv := 1.0 / len
	return Vec3{v.x * inv, v.y * inv, v.z * inv}
}

vec3_distance :: proc "contextless" (a, b: Vec3) -> f32 {
	return vec3_length(a - b)
}

vec3_distance_sq :: proc "contextless" (a, b: Vec3) -> f32 {
	return vec3_length_sq(a - b)
}

vec3_lerp :: proc "contextless" (a, b: Vec3, t: f32) -> Vec3 {
	return a + (b - a) * t
}

//
// Quaternion operations.
//

quat_identity :: proc "contextless" () -> Quat {
	return Quat{w = 1, x = 0, y = 0, z = 0}
}

quat_length_sq :: proc "contextless" (q: Quat) -> f32 {
	return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z
}

quat_length :: proc "contextless" (q: Quat) -> f32 {
	return math.sqrt(quat_length_sq(q))
}

quat_normalize :: proc "contextless" (q: Quat) -> Quat {
	len := quat_length(q)
	if len < 1e-8 {
		return quat_identity()
	}
	inv := 1.0 / len
	return Quat{q.w * inv, q.x * inv, q.y * inv, q.z * inv}
}

quat_conjugate :: proc "contextless" (q: Quat) -> Quat {
	return Quat{w = q.w, x = -q.x, y = -q.y, z = -q.z}
}

quat_inverse :: proc "contextless" (q: Quat) -> Quat {
	len_sq := quat_length_sq(q)
	if len_sq < 1e-8 {
		return quat_identity()
	}
	inv := 1.0 / len_sq
	return Quat{q.w * inv, -q.x * inv, -q.y * inv, -q.z * inv}
}

quat_mul :: proc "contextless" (a, b: Quat) -> Quat {
	return Quat{
		w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	}
}

quat_from_axis_angle :: proc "contextless" (axis: Vec3, angle_radians: f32) -> Quat {
	half_angle := angle_radians * 0.5
	s := math.sin(half_angle)
	c := math.cos(half_angle)
	norm_axis := vec3_normalize(axis)
	return Quat{
		w = c,
		x = norm_axis.x * s,
		y = norm_axis.y * s,
		z = norm_axis.z * s,
	}
}

quat_from_euler_xyz :: proc "contextless" (roll, pitch, yaw: f32) -> Quat {
	hr := roll * 0.5
	hp := pitch * 0.5
	hy := yaw * 0.5

	cr := math.cos(hr)
	sr := math.sin(hr)
	cp := math.cos(hp)
	sp := math.sin(hp)
	cy := math.cos(hy)
	sy := math.sin(hy)

	return Quat{
		w = cr * cp * cy + sr * sp * sy,
		x = sr * cp * cy - cr * sp * sy,
		y = cr * sp * cy + sr * cp * sy,
		z = cr * cp * sy - sr * sp * cy,
	}
}

quat_to_euler_xyz :: proc "contextless" (q: Quat) -> (roll, pitch, yaw: f32) {
	// Roll (X-axis rotation)
	sinr_cosp := 2.0 * (q.w * q.x + q.y * q.z)
	cosr_cosp := 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
	roll = math.atan2(sinr_cosp, cosr_cosp)

	// Pitch (Y-axis rotation)
	sinp := 2.0 * (q.w * q.y - q.z * q.x)
	if math.abs(sinp) >= 1.0 {
		pitch = math.copy_sign(f32(math.PI * 0.5), sinp)
	} else {
		pitch = math.asin(sinp)
	}

	// Yaw (Z-axis rotation)
	siny_cosp := 2.0 * (q.w * q.z + q.x * q.y)
	cosy_cosp := 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
	yaw = math.atan2(siny_cosp, cosy_cosp)
	return
}

quat_rotate_vec3 :: proc "contextless" (q: Quat, v: Vec3) -> Vec3 {
	// Formula: v' = v + 2 * r x (r x v + w * v) where r = (x, y, z)
	r := Vec3{q.x, q.y, q.z}
	t := vec3_cross(r, v) * 2.0
	return v + t * q.w + vec3_cross(r, t)
}

quat_slerp :: proc "contextless" (q0, q1: Quat, t: f32) -> Quat {
	cos_half_theta := q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z
	target := q1

	// If dot product is negative, reverse target to take shortest path
	if cos_half_theta < 0.0 {
		target = Quat{-q1.w, -q1.x, -q1.y, -q1.z}
		cos_half_theta = -cos_half_theta
	}

	if cos_half_theta > 0.9995 {
		// Linear interpolation when quaternions are extremely close to avoid division by zero
		lerped := Quat{
			q0.w + (target.w - q0.w) * t,
			q0.x + (target.x - q0.x) * t,
			q0.y + (target.y - q0.y) * t,
			q0.z + (target.z - q0.z) * t,
		}
		return quat_normalize(lerped)
	}

	half_theta := math.acos(cos_half_theta)
	sin_half_theta := math.sqrt(1.0 - cos_half_theta * cos_half_theta)

	ratio_a := math.sin((1.0 - t) * half_theta) / sin_half_theta
	ratio_b := math.sin(t * half_theta) / sin_half_theta

	return Quat{
		q0.w * ratio_a + target.w * ratio_b,
		q0.x * ratio_a + target.x * ratio_b,
		q0.y * ratio_a + target.y * ratio_b,
		q0.z * ratio_a + target.z * ratio_b,
	}
}

//
// Matrix conversion routines.
//

mat3_identity :: proc "contextless" () -> Mat3 {
	return Mat3{
		1, 0, 0,
		0, 1, 0,
		0, 0, 1,
	}
}

mat4_identity :: proc "contextless" () -> Mat4 {
	return Mat4{
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	}
}

quat_to_mat3 :: proc "contextless" (q: Quat) -> Mat3 {
	xx := q.x * q.x
	yy := q.y * q.y
	zz := q.z * q.z
	xy := q.x * q.y
	xz := q.x * q.z
	yz := q.y * q.z
	wx := q.w * q.x
	wy := q.w * q.y
	wz := q.w * q.z

	return Mat3{
		1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy),
		2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
		2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy),
	}
}

mat3_to_quat :: proc "contextless" (m: Mat3) -> Quat {
	trace := m[0, 0] + m[1, 1] + m[2, 2]
	if trace > 0.0 {
		s := 0.5 / math.sqrt(trace + 1.0)
		return quat_normalize(Quat{
			w = 0.25 / s,
			x = (m[2, 1] - m[1, 2]) * s,
			y = (m[0, 2] - m[2, 0]) * s,
			z = (m[1, 0] - m[0, 1]) * s,
		})
	} else if m[0, 0] > m[1, 1] && m[0, 0] > m[2, 2] {
		s := 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
		return quat_normalize(Quat{
			w = (m[2, 1] - m[1, 2]) / s,
			x = 0.25 * s,
			y = (m[0, 1] + m[1, 0]) / s,
			z = (m[0, 2] + m[2, 0]) / s,
		})
	} else if m[1, 1] > m[2, 2] {
		s := 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
		return quat_normalize(Quat{
			w = (m[0, 2] - m[2, 0]) / s,
			x = (m[0, 1] + m[1, 0]) / s,
			y = 0.25 * s,
			z = (m[1, 2] + m[2, 1]) / s,
		})
	} else {
		s := 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
		return quat_normalize(Quat{
			w = (m[1, 0] - m[0, 1]) / s,
			x = (m[0, 2] + m[2, 0]) / s,
			y = (m[1, 2] + m[2, 1]) / s,
			z = 0.25 * s,
		})
	}
}

transform3d_identity :: proc "contextless" () -> Transform3D {
	return Transform3D{
		translation = vec3_zero(),
		rotation    = quat_identity(),
	}
}

transform3d_to_mat4 :: proc "contextless" (tf: Transform3D) -> Mat4 {
	r := quat_to_mat3(tf.rotation)
	t := tf.translation
	return Mat4{
		r[0, 0], r[0, 1], r[0, 2], t.x,
		r[1, 0], r[1, 1], r[1, 2], t.y,
		r[2, 0], r[2, 1], r[2, 2], t.z,
		0,       0,       0,       1,
	}
}

transform3d_apply_point :: proc "contextless" (tf: Transform3D, p: Vec3) -> Vec3 {
	return quat_rotate_vec3(tf.rotation, p) + tf.translation
}

transform3d_apply_vector :: proc "contextless" (tf: Transform3D, v: Vec3) -> Vec3 {
	return quat_rotate_vec3(tf.rotation, v)
}

transform3d_inverse :: proc "contextless" (tf: Transform3D) -> Transform3D {
	inv_rot := quat_inverse(tf.rotation)
	inv_trans := quat_rotate_vec3(inv_rot, -tf.translation)
	return Transform3D{
		translation = inv_trans,
		rotation    = inv_rot,
	}
}

transform3d_mul :: proc "contextless" (a, b: Transform3D) -> Transform3D {
	return Transform3D{
		translation = transform3d_apply_point(a, b.translation),
		rotation    = quat_mul(a.rotation, b.rotation),
	}
}
