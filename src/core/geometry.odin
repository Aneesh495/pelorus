package pelorus_core

import "core:math"

//
// 3D Ray parameterized as r(t) = origin + t * direction.
// Precomputes inverse direction for fast branchless slab intersection tests.
//
Ray3D :: struct {
	origin:    Vec3,
	direction: Vec3, // Unit normalized direction
	inv_dir:   Vec3, // Component-wise reciprocal: 1.0 / direction
}

//
// 3D Plane represented in Hesse normal form: dot(normal, p) + distance = 0.
//
Plane3D :: struct {
	normal:   Vec3,
	distance: f32,
}

//
// 3D Bounding Sphere.
//
Sphere3D :: struct {
	center: Vec3,
	radius: f32,
}

//
// 3D Capsule primitive: line segment between start and end swept by radius.
//
Capsule3D :: struct {
	segment_start: Vec3,
	segment_end:   Vec3,
	radius:        f32,
}

//
// 3D Axis-Aligned Bounding Box (AABB).
//
AABB :: struct {
	min_pt: Vec3,
	max_pt: Vec3,
}

//
// 3D Oriented Bounding Box (OBB).
//
OBB :: struct {
	center:       Vec3,
	axes:         [3]Vec3, // Orthogonal unit orientation axes (local X, Y, Z)
	half_extents: Vec3,    // Extents along each axis from center
	yaw:          f32,     // Planar heading angle (radians)
}

//
// Ray3D constructor with automatic inverse direction caching.
//
ray3d_make :: proc "contextless" (origin, direction: Vec3) -> Ray3D {
	norm_dir := vec3_normalize(direction)
	inv_x := 1.0 / (norm_dir.x != 0.0 ? norm_dir.x : 1e-9)
	inv_y := 1.0 / (norm_dir.y != 0.0 ? norm_dir.y : 1e-9)
	inv_z := 1.0 / (norm_dir.z != 0.0 ? norm_dir.z : 1e-9)
	return Ray3D{
		origin    = origin,
		direction = norm_dir,
		inv_dir   = Vec3{inv_x, inv_y, inv_z},
	}
}

//
// Evaluate point along ray at distance t: p = origin + t * direction.
//
ray3d_at :: proc "contextless" (ray: Ray3D, t: f32) -> Vec3 {
	return ray.origin + ray.direction * t
}

//
// Plane3D constructor from normal and arbitrary point on plane.
//
plane3d_from_point_normal :: proc "contextless" (point, normal: Vec3) -> Plane3D {
	n := vec3_normalize(normal)
	d := -vec3_dot(n, point)
	return Plane3D{normal = n, distance = d}
}

//
// Signed distance from point to plane.
// Positive when on side of normal, negative behind.
//
plane3d_signed_distance :: proc "contextless" (plane: Plane3D, point: Vec3) -> f32 {
	return vec3_dot(plane.normal, point) + plane.distance
}

//
// AABB constructor from min and max points.
//
aabb_make :: proc "contextless" (min_pt, max_pt: Vec3) -> AABB {
	return AABB{min_pt = min_pt, max_pt = max_pt}
}

//
// Empty AABB initialized to extreme limits.
//
aabb_empty :: proc "contextless" () -> AABB {
	inf := f32(1e30)
	return AABB{
		min_pt = Vec3{inf, inf, inf},
		max_pt = Vec3{-inf, -inf, -inf},
	}
}

//
// Geometric center point of AABB.
//
aabb_center :: proc "contextless" (box: AABB) -> Vec3 {
	return (box.min_pt + box.max_pt) * 0.5
}

//
// Full dimensional extents (width, length, height) of AABB.
//
aabb_extents :: proc "contextless" (box: AABB) -> Vec3 {
	return box.max_pt - box.min_pt
}

//
// Half-extents of AABB from center.
//
aabb_half_extents :: proc "contextless" (box: AABB) -> Vec3 {
	return (box.max_pt - box.min_pt) * 0.5
}

//
// Volume of AABB.
//
aabb_volume :: proc "contextless" (box: AABB) -> f32 {
	ext := aabb_extents(box)
	if ext.x <= 0 || ext.y <= 0 || ext.z <= 0 do return 0.0
	return ext.x * ext.y * ext.z
}

//
// Total surface area of AABB.
//
aabb_surface_area :: proc "contextless" (box: AABB) -> f32 {
	ext := aabb_extents(box)
	if ext.x <= 0 || ext.y <= 0 || ext.z <= 0 do return 0.0
	return 2.0 * (ext.x * ext.y + ext.y * ext.z + ext.z * ext.x)
}

//
// Test if point lies inside AABB.
//
aabb_contains_point :: proc "contextless" (box: AABB, p: Vec3) -> bool {
	return p.x >= box.min_pt.x && p.x <= box.max_pt.x &&
	       p.y >= box.min_pt.y && p.y <= box.max_pt.y &&
	       p.z >= box.min_pt.z && p.z <= box.max_pt.z
}

//
// Expand AABB to encompass a given point.
//
aabb_expand_point :: proc "contextless" (box: ^AABB, p: Vec3) {
	box.min_pt.x = math.min(box.min_pt.x, p.x)
	box.min_pt.y = math.min(box.min_pt.y, p.y)
	box.min_pt.z = math.min(box.min_pt.z, p.z)

	box.max_pt.x = math.max(box.max_pt.x, p.x)
	box.max_pt.y = math.max(box.max_pt.y, p.y)
	box.max_pt.z = math.max(box.max_pt.z, p.z)
}

//
// Merge two AABB boxes into minimal bounding box enclosing both.
//
aabb_merge :: proc "contextless" (a, b: AABB) -> AABB {
	return AABB{
		min_pt = Vec3{
			math.min(a.min_pt.x, b.min_pt.x),
			math.min(a.min_pt.y, b.min_pt.y),
			math.min(a.min_pt.z, b.min_pt.z),
		},
		max_pt = Vec3{
			math.max(a.max_pt.x, b.max_pt.x),
			math.max(a.max_pt.y, b.max_pt.y),
			math.max(a.max_pt.z, b.max_pt.z),
		},
	}
}

//
// Test intersection between two AABBs.
//
aabb_intersects_aabb :: proc "contextless" (a, b: AABB) -> bool {
	if a.max_pt.x < b.min_pt.x || a.min_pt.x > b.max_pt.x do return false
	if a.max_pt.y < b.min_pt.y || a.min_pt.y > b.max_pt.y do return false
	if a.max_pt.z < b.min_pt.z || a.min_pt.z > b.max_pt.z do return false
	return true
}

//
// Fast branchless slab intersection between Ray and AABB.
// Returns true if ray hits box within [t_min, t_max], along with entry and exit distances.
//
aabb_intersects_ray :: proc "contextless" (
	box: AABB,
	ray: Ray3D,
	t_min_in: f32 = 0.0,
	t_max_in: f32 = 1e30,
) -> (hit: bool, t_entry: f32, t_exit: f32) {
	tx1 := (box.min_pt.x - ray.origin.x) * ray.inv_dir.x
	tx2 := (box.max_pt.x - ray.origin.x) * ray.inv_dir.x
	tmin := math.min(tx1, tx2)
	tmax := math.max(tx1, tx2)

	ty1 := (box.min_pt.y - ray.origin.y) * ray.inv_dir.y
	ty2 := (box.max_pt.y - ray.origin.y) * ray.inv_dir.y
	tmin = math.max(tmin, math.min(ty1, ty2))
	tmax = math.min(tmax, math.max(ty1, ty2))

	tz1 := (box.min_pt.z - ray.origin.z) * ray.inv_dir.z
	tz2 := (box.max_pt.z - ray.origin.z) * ray.inv_dir.z
	tmin = math.max(tmin, math.min(tz1, tz2))
	tmax = math.min(tmax, math.max(tz1, tz2))

	t_entry = math.max(tmin, t_min_in)
	t_exit = math.min(tmax, t_max_in)

	hit = (t_exit >= t_entry) && (t_exit >= 0.0)
	return
}

//
// OBB constructor from center, dimensional size (length, width, height), and yaw heading.
//
obb_from_center_size_yaw :: proc "contextless" (center: Vec3, size: Vec3, yaw: f32) -> OBB {
	cy := math.cos(yaw)
	sy := math.sin(yaw)

	axis_x := Vec3{cy, sy, 0.0}
	axis_y := Vec3{-sy, cy, 0.0}
	axis_z := Vec3{0.0, 0.0, 1.0}

	return OBB{
		center       = center,
		axes         = [3]Vec3{axis_x, axis_y, axis_z},
		half_extents = size * 0.5,
		yaw          = yaw,
	}
}

//
// Extract the 8 corner vertices of an OBB in world coordinates.
//
obb_corners :: proc "contextless" (box: OBB) -> [8]Vec3 {
	ax := box.axes[0] * box.half_extents.x
	ay := box.axes[1] * box.half_extents.y
	az := box.axes[2] * box.half_extents.z

	c := box.center
	return [8]Vec3{
		c - ax - ay - az,
		c + ax - ay - az,
		c + ax + ay - az,
		c - ax + ay - az,
		c - ax - ay + az,
		c + ax - ay + az,
		c + ax + ay + az,
		c - ax + ay + az,
	}
}

//
// Convert world-space point into local coordinate frame of OBB.
//
obb_world_to_local :: proc "contextless" (box: OBB, p: Vec3) -> Vec3 {
	d := p - box.center
	return Vec3{
		vec3_dot(d, box.axes[0]),
		vec3_dot(d, box.axes[1]),
		vec3_dot(d, box.axes[2]),
	}
}

//
// Test if point is contained inside 3D OBB.
//
obb_contains_point :: proc "contextless" (box: OBB, p: Vec3) -> bool {
	local_p := obb_world_to_local(box, p)
	return math.abs(local_p.x) <= box.half_extents.x &&
	       math.abs(local_p.y) <= box.half_extents.y &&
	       math.abs(local_p.z) <= box.half_extents.z
}

//
// Separating Axis Theorem (SAT) implementation for 3D OBB vs OBB overlap detection.
// Tests 15 potential separating axes: 3 face normals of A, 3 face normals of B, and
// 9 pairwise cross-products of edges.
//
obb_intersects_obb :: proc "contextless" (a, b: OBB) -> bool {
	// Translation vector from center A to center B
	t := b.center - a.center

	// Rotation matrix expressing B in A's coordinate frame: R_{ij} = dot(A_i, B_j)
	r: [3][3]f32
	abs_r: [3][3]f32
	for i in 0 ..< 3 {
		for j in 0 ..< 3 {
			r[i][j] = vec3_dot(a.axes[i], b.axes[j])
			// Add small epsilon to prevent parallel edge division issues in cross products
			abs_r[i][j] = math.abs(r[i][j]) + 1e-5
		}
	}

	// Translation in A's frame
	t_a := Vec3{
		vec3_dot(t, a.axes[0]),
		vec3_dot(t, a.axes[1]),
		vec3_dot(t, a.axes[2]),
	}

	// 1. Test A's coordinate axes (A0, A1, A2)
	for i in 0 ..< 3 {
		ra := (i == 0) ? a.half_extents.x : ((i == 1) ? a.half_extents.y : a.half_extents.z)
		rb := b.half_extents.x * abs_r[i][0] + b.half_extents.y * abs_r[i][1] + b.half_extents.z * abs_r[i][2]
		trans_proj := (i == 0) ? math.abs(t_a.x) : ((i == 1) ? math.abs(t_a.y) : math.abs(t_a.z))
		if trans_proj > (ra + rb) do return false
	}

	// 2. Test B's coordinate axes (B0, B1, B2)
	for j in 0 ..< 3 {
		ra := a.half_extents.x * abs_r[0][j] + a.half_extents.y * abs_r[1][j] + a.half_extents.z * abs_r[2][j]
		rb := (j == 0) ? b.half_extents.x : ((j == 1) ? b.half_extents.y : b.half_extents.z)
		trans_proj := math.abs(t_a.x * r[0][j] + t_a.y * r[1][j] + t_a.z * r[2][j])
		if trans_proj > (ra + rb) do return false
	}

	// 3. Test 9 cross-product axes: A_i x B_j
	// Axis A0 x B0
	ra := a.half_extents.y * abs_r[2][0] + a.half_extents.z * abs_r[1][0]
	rb := b.half_extents.y * abs_r[0][2] + b.half_extents.z * abs_r[0][1]
	if math.abs(t_a.z * r[1][0] - t_a.y * r[2][0]) > (ra + rb) do return false

	// Axis A0 x B1
	ra = a.half_extents.y * abs_r[2][1] + a.half_extents.z * abs_r[1][1]
	rb = b.half_extents.x * abs_r[0][2] + b.half_extents.z * abs_r[0][0]
	if math.abs(t_a.z * r[1][1] - t_a.y * r[2][1]) > (ra + rb) do return false

	// Axis A0 x B2
	ra = a.half_extents.y * abs_r[2][2] + a.half_extents.z * abs_r[1][2]
	rb = b.half_extents.x * abs_r[0][1] + b.half_extents.y * abs_r[0][0]
	if math.abs(t_a.z * r[1][2] - t_a.y * r[2][2]) > (ra + rb) do return false

	// Axis A1 x B0
	ra = a.half_extents.x * abs_r[2][0] + a.half_extents.z * abs_r[0][0]
	rb = b.half_extents.y * abs_r[1][2] + b.half_extents.z * abs_r[1][1]
	if math.abs(t_a.x * r[2][0] - t_a.z * r[0][0]) > (ra + rb) do return false

	// Axis A1 x B1
	ra = a.half_extents.x * abs_r[2][1] + a.half_extents.z * abs_r[0][1]
	rb = b.half_extents.x * abs_r[1][2] + b.half_extents.z * abs_r[1][0]
	if math.abs(t_a.x * r[2][1] - t_a.z * r[0][1]) > (ra + rb) do return false

	// Axis A1 x B2
	ra = a.half_extents.x * abs_r[2][2] + a.half_extents.z * abs_r[0][2]
	rb = b.half_extents.x * abs_r[1][1] + b.half_extents.y * abs_r[1][0]
	if math.abs(t_a.x * r[2][2] - t_a.z * r[0][2]) > (ra + rb) do return false

	// Axis A2 x B0
	ra = a.half_extents.x * abs_r[1][0] + a.half_extents.y * abs_r[0][0]
	rb = b.half_extents.y * abs_r[2][2] + b.half_extents.z * abs_r[2][1]
	if math.abs(t_a.y * r[0][0] - t_a.x * r[1][0]) > (ra + rb) do return false

	// Axis A2 x B1
	ra = a.half_extents.x * abs_r[1][1] + a.half_extents.y * abs_r[0][1]
	rb = b.half_extents.x * abs_r[2][2] + b.half_extents.z * abs_r[2][0]
	if math.abs(t_a.y * r[0][1] - t_a.x * r[1][1]) > (ra + rb) do return false

	// Axis A2 x B2
	ra = a.half_extents.x * abs_r[1][2] + a.half_extents.y * abs_r[0][2]
	rb = b.half_extents.x * abs_r[2][1] + b.half_extents.y * abs_r[2][0]
	if math.abs(t_a.y * r[0][2] - t_a.x * r[1][2]) > (ra + rb) do return false

	// No separating axis exists: boxes overlap
	return true
}

//
// Spatial hash cell for local neighborhood queries.
//
SpatialNeighborCell :: struct {
	key:         u64,
	point_index: int,
	next_in_bin: int,
	occupied:    bool,
}

//
// SplitMix64 integer hash for combining 3D discrete grid coordinates (gx, gy, gz) into 64-bit key.
//
voxel_hash_coords :: proc "contextless" (gx, gy, gz: i32) -> u64 {
	h := u64(u32(gx)) * 73856093 ~ u64(u32(gy)) * 19349663 ~ u64(u32(gz)) * 83492791
	h = (h ~ (h >> 30)) * 0xbf58476d1ce4e5b9
	h = (h ~ (h >> 27)) * 0x94d049bb133111eb
	h = h ~ (h >> 31)
	return h | 1 // Guarantee non-zero key
}

//
// Closest distance between two line segments (used in capsule collision queries).
// Returns squared distance between segment S1 (p1 to q1) and S2 (p2 to q2).
//
segment_segment_distance_sq :: proc "contextless" (p1, q1, p2, q2: Vec3) -> f32 {
	d1 := q1 - p1 // Direction vector of segment S1
	d2 := q2 - p2 // Direction vector of segment S2
	r := p1 - p2

	a := vec3_dot(d1, d1) // Squared length of S1
	e := vec3_dot(d2, d2) // Squared length of S2
	f := vec3_dot(d2, r)

	s: f32 = 0.0
	t: f32 = 0.0

	// Check if either or both segments degenerate into points
	if a <= 1e-6 && e <= 1e-6 {
		return vec3_dot(r, r)
	}
	if a <= 1e-6 {
		s = 0.0
		t = math.clamp(f / e, 0.0, 1.0)
	} else {
		c := vec3_dot(d1, r)
		if e <= 1e-6 {
			t = 0.0
			s = math.clamp(-c / a, 0.0, 1.0)
		} else {
			// General nondegenerate case
			b := vec3_dot(d1, d2)
			denom := a * e - b * b

			if denom != 0.0 {
				s = math.clamp((b * f - c * e) / denom, 0.0, 1.0)
			} else {
				s = 0.0
			}

			t = (b * s + f) / e
			if t < 0.0 {
				t = 0.0
				s = math.clamp(-c / a, 0.0, 1.0)
			} else if t > 1.0 {
				t = 1.0
				s = math.clamp((b - c) / a, 0.0, 1.0)
			}
		}
	}

	closest1 := p1 + d1 * s
	closest2 := p2 + d2 * t
	return vec3_distance_sq(closest1, closest2)
}

//
// Test intersection between two 3D Capsule primitives.
//
capsule_intersects_capsule :: proc "contextless" (c1, c2: Capsule3D) -> bool {
	dist_sq := segment_segment_distance_sq(c1.segment_start, c1.segment_end, c2.segment_start, c2.segment_end)
	sum_radii := c1.radius + c2.radius
	return dist_sq <= (sum_radii * sum_radii)
}
