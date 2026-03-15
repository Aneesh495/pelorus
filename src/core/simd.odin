package pelorus_core

import "core:math"
import "core:simd"

//
// 128-bit SIMD vector containing four 32-bit single-precision floats.
//
SimdVec4 :: simd.f32x4

//
// Construct a SIMD 4-float vector from four scalar values.
//
simd_vec4_set :: proc "contextless" (x, y, z, w: f32) -> SimdVec4 {
	arr := [4]f32{x, y, z, w}
	return simd.from_array(arr)
}

//
// Construct a SIMD 4-float vector by broadcasting a single scalar across all lanes.
//
simd_vec4_broadcast :: proc "contextless" (val: f32) -> SimdVec4 {
	arr := [4]f32{val, val, val, val}
	return simd.from_array(arr)
}

//
// Convert a SIMD vector back to an array of 4 floats.
//
simd_vec4_to_array :: proc "contextless" (v: SimdVec4) -> [4]f32 {
	return simd.to_array(v)
}

//
// Load a 3D vector into a SIMD register with the fourth lane set to zero.
//
simd_vec4_from_vec3 :: proc "contextless" (v: Vec3, w: f32 = 0.0) -> SimdVec4 {
	arr := [4]f32{v.x, v.y, v.z, w}
	return simd.from_array(arr)
}

//
// Extract the first three lanes into a standard Vec3.
//
simd_vec4_to_vec3 :: proc "contextless" (v: SimdVec4) -> Vec3 {
	arr := simd.to_array(v)
	return Vec3{arr[0], arr[1], arr[2]}
}

//
// Component-wise addition of two SIMD vectors.
//
simd_add :: proc "contextless" (a, b: SimdVec4) -> SimdVec4 {
	return a + b
}

//
// Component-wise subtraction of two SIMD vectors.
//
simd_sub :: proc "contextless" (a, b: SimdVec4) -> SimdVec4 {
	return a - b
}

//
// Component-wise multiplication of two SIMD vectors.
//
simd_mul :: proc "contextless" (a, b: SimdVec4) -> SimdVec4 {
	return a * b
}

//
// Fused multiply-add: computes (a * b) + c.
//
simd_fma :: proc "contextless" (a, b, c: SimdVec4) -> SimdVec4 {
	return (a * b) + c
}

//
// Component-wise minimum across lanes.
//
simd_min :: proc "contextless" (a, b: SimdVec4) -> SimdVec4 {
	arr_a := simd.to_array(a)
	arr_b := simd.to_array(b)
	return simd.from_array([4]f32{
		math.min(arr_a[0], arr_b[0]),
		math.min(arr_a[1], arr_b[1]),
		math.min(arr_a[2], arr_b[2]),
		math.min(arr_a[3], arr_b[3]),
	})
}

//
// Component-wise maximum across lanes.
//
simd_max :: proc "contextless" (a, b: SimdVec4) -> SimdVec4 {
	arr_a := simd.to_array(a)
	arr_b := simd.to_array(b)
	return simd.from_array([4]f32{
		math.max(arr_a[0], arr_b[0]),
		math.max(arr_a[1], arr_b[1]),
		math.max(arr_a[2], arr_b[2]),
		math.max(arr_a[3], arr_b[3]),
	})
}

//
// Compute 3D dot product of two vectors loaded in SIMD registers (ignoring lane 3).
//
simd_dot3 :: proc "contextless" (a, b: SimdVec4) -> f32 {
	prod := simd.to_array(a * b)
	return prod[0] + prod[1] + prod[2]
}

//
// Compute full 4D dot product across all four lanes.
//
simd_dot4 :: proc "contextless" (a, b: SimdVec4) -> f32 {
	prod := simd.to_array(a * b)
	return prod[0] + prod[1] + prod[2] + prod[3]
}

//
// Vectorized batch point transformation: transforms N 3D points by a 4x4 matrix.
//
simd_transform_points_batch :: proc(
	points_out: []Vec3,
	points_in: []Vec3,
	matrix_transform: Mat4,
) {
	count := math.min(len(points_out), len(points_in))
	if count == 0 do return

	col0 := simd_vec4_set(matrix_transform[0, 0], matrix_transform[1, 0], matrix_transform[2, 0], 0.0)
	col1 := simd_vec4_set(matrix_transform[0, 1], matrix_transform[1, 1], matrix_transform[2, 1], 0.0)
	col2 := simd_vec4_set(matrix_transform[0, 2], matrix_transform[1, 2], matrix_transform[2, 2], 0.0)
	col3 := simd_vec4_set(matrix_transform[0, 3], matrix_transform[1, 3], matrix_transform[2, 3], 0.0)

	for i in 0 ..< count {
		p := points_in[i]
		px := simd_vec4_broadcast(p.x)
		py := simd_vec4_broadcast(p.y)
		pz := simd_vec4_broadcast(p.z)

		// res = col0 * px + col1 * py + col2 * pz + col3
		res := col3
		res = simd_fma(col0, px, res)
		res = simd_fma(col1, py, res)
		res = simd_fma(col2, pz, res)

		points_out[i] = simd_vec4_to_vec3(res)
	}
}

//
// Vectorized computation of minimum and maximum spatial bounds across a point buffer.
//
simd_compute_bounds :: proc(points: []Vec3) -> (min_bound: Vec3, max_bound: Vec3) {
	if len(points) == 0 {
		return vec3_zero(), vec3_zero()
	}

	simd_min_acc := simd_vec4_from_vec3(points[0])
	simd_max_acc := simd_vec4_from_vec3(points[0])

	for i in 1 ..< len(points) {
		p_simd := simd_vec4_from_vec3(points[i])
		simd_min_acc = simd_min(simd_min_acc, p_simd)
		simd_max_acc = simd_max(simd_max_acc, p_simd)
	}

	min_bound = simd_vec4_to_vec3(simd_min_acc)
	max_bound = simd_vec4_to_vec3(simd_max_acc)
	return
}

//
// Vectorized squared distance batch evaluation: fills distances_sq with squared distance
// from query_point to each point in the target points slice.
//
simd_batch_squared_distances :: proc(
	distances_sq: []f32,
	query_point: Vec3,
	points: []Vec3,
) {
	count := math.min(len(distances_sq), len(points))
	q_simd := simd_vec4_from_vec3(query_point)

	for i in 0 ..< count {
		p_simd := simd_vec4_from_vec3(points[i])
		diff := p_simd - q_simd
		diff_sq := diff * diff
		arr := simd.to_array(diff_sq)
		distances_sq[i] = arr[0] + arr[1] + arr[2]
	}
}
