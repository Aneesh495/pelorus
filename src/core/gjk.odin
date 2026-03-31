package pelorus_core

import "core:math"

//
// Maximum number of iterations for GJK convergence.
//
GJK_MAX_ITERATIONS :: 32
GJK_EPSILON :: 1e-5

//
// Convex shape interface tag for support mapping.
//
ConvexKind :: enum u8 {
	Sphere,
	Box,
	Capsule,
	PointSet,
}

//
// Generic Convex Shape wrapper for GJK collision queries.
//
ConvexShape :: struct {
	kind:      ConvexKind,
	transform: Transform3D,
	// Primitive specific parameters
	sphere:    struct { radius: f32 },
	box:       struct { half_extents: Vec3 },
	capsule:   struct { length: f32, radius: f32 },
	points:    []Vec3, // For arbitrary convex hulls / point clusters
}

//
// Simplex structure maintaining up to 4 vertices in Minkowski difference space.
//
GjkSimplex :: struct {
	points: [4]Vec3,
	count:  int,
}

//
// Support mapping S_A(direction) for ConvexShape in world coordinates.
//
convex_support :: proc "contextless" (shape: ConvexShape, dir: Vec3) -> Vec3 {
	norm_dir := vec3_normalize(dir)
	// Transform direction into local shape coordinate space
	inv_rot := quat_inverse(shape.transform.rotation)
	local_dir := quat_rotate_vec3(inv_rot, norm_dir)

	local_support: Vec3

	switch shape.kind {
	case .Sphere:
		local_support = local_dir * shape.sphere.radius

	case .Box:
		local_support = Vec3{
			local_dir.x >= 0.0 ? shape.box.half_extents.x : -shape.box.half_extents.x,
			local_dir.y >= 0.0 ? shape.box.half_extents.y : -shape.box.half_extents.y,
			local_dir.z >= 0.0 ? shape.box.half_extents.z : -shape.box.half_extents.z,
		}

	case .Capsule:
		half_len := shape.capsule.length * 0.5
		base_pt := Vec3{0, 0, local_dir.z >= 0.0 ? half_len : -half_len}
		local_support = base_pt + local_dir * shape.capsule.radius

	case .PointSet:
		if len(shape.points) == 0 {
			local_support = vec3_zero()
		} else {
			best_dot := vec3_dot(shape.points[0], local_dir)
			best_idx := 0
			for i in 1 ..< len(shape.points) {
				d := vec3_dot(shape.points[i], local_dir)
				if d > best_dot {
					best_dot = d
					best_idx = i
				}
			}
			local_support = shape.points[best_idx]
		}
	}

	// Transform local support point back to world space
	return transform3d_apply_point(shape.transform, local_support)
}

//
// Minkowski difference support mapping: S_{A - B}(dir) = S_A(dir) - S_B(-dir).
//
minkowski_support :: proc "contextless" (a, b: ConvexShape, dir: Vec3) -> Vec3 {
	return convex_support(a, dir) - convex_support(b, -dir)
}

//
// Push point onto GJK simplex.
//
gjk_simplex_push :: proc "contextless" (simplex: ^GjkSimplex, p: Vec3) {
	if simplex.count < 4 {
		simplex.points[simplex.count] = p
		simplex.count += 1
	}
}

//
// Simplex solver: examines the current simplex, determines the Voronoi region
// containing the origin, updates the simplex to the minimal face closest to origin,
// and sets the next search direction toward the origin.
// Returns true if the simplex contains the origin (penetration / intersection).
//
gjk_do_simplex :: proc "contextless" (simplex: ^GjkSimplex, dir: ^Vec3) -> bool {
	switch simplex.count {
	case 2:
		// Line segment AB: simplex.points[1] is A (most recent), simplex.points[0] is B
		a := simplex.points[1]
		b := simplex.points[0]
		ab := b - a
		ao := -a

		if vec3_dot(ab, ao) > 0.0 {
			// Origin is in region of edge AB
			dir^ = vec3_cross(vec3_cross(ab, ao), ab)
			if vec3_length_sq(dir^) < 1e-8 {
				dir^ = Vec3{-ab.y, ab.x, 0.0}
			}
		} else {
			// Origin is closest to vertex A
			simplex.points[0] = a
			simplex.count = 1
			dir^ = ao
		}
		return false

	case 3:
		// Triangle ABC: simplex.points[2] is A (most recent), [1] is B, [0] is C
		a := simplex.points[2]
		b := simplex.points[1]
		c := simplex.points[0]

		ab := b - a
		ac := c - a
		ao := -a
		abc := vec3_cross(ab, ac) // Triangle face normal

		// Normal to AB pointing away from C
		ab_perp := vec3_cross(ab, abc)
		if vec3_dot(ab_perp, ao) > 0.0 {
			// Origin is outside edge AB
			simplex.points[0] = b
			simplex.points[1] = a
			simplex.count = 2
			dir^ = vec3_cross(vec3_cross(ab, ao), ab)
			return false
		}

		// Normal to AC pointing away from B
		ac_perp := vec3_cross(abc, ac)
		if vec3_dot(ac_perp, ao) > 0.0 {
			// Origin is outside edge AC
			simplex.points[0] = c
			simplex.points[1] = a
			simplex.count = 2
			dir^ = vec3_cross(vec3_cross(ac, ao), ac)
			return false
		}

		// Origin is above or below triangle ABC
		if vec3_dot(abc, ao) > 0.0 {
			dir^ = abc
		} else {
			// Reverse triangle orientation to keep winding consistent
			simplex.points[0] = b
			simplex.points[1] = c
			dir^ = -abc
		}
		return false

	case 4:
		// Tetrahedron ABCD: simplex.points[3] is A (most recent), [2] is B, [1] is C, [0] is D
		a := simplex.points[3]
		b := simplex.points[2]
		c := simplex.points[1]
		d := simplex.points[0]

		ab := b - a
		ac := c - a
		ad := d - a
		ao := -a

		// Face normals pointing outward
		abc := vec3_cross(ab, ac)
		acd := vec3_cross(ac, ad)
		adb := vec3_cross(ad, ab)

		// Check if face normals point toward D; if so, invert to point outward
		if vec3_dot(abc, ad) > 0.0 do abc = -abc
		if vec3_dot(acd, ab) > 0.0 do acd = -acd
		if vec3_dot(adb, ac) > 0.0 do adb = -adb

		if vec3_dot(abc, ao) > 0.0 {
			// Origin outside face ABC
			simplex.points[0] = c
			simplex.points[1] = b
			simplex.points[2] = a
			simplex.count = 3
			dir^ = abc
			return false
		}

		if vec3_dot(acd, ao) > 0.0 {
			// Origin outside face ACD
			simplex.points[0] = d
			simplex.points[1] = c
			simplex.points[2] = a
			simplex.count = 3
			dir^ = acd
			return false
		}

		if vec3_dot(adb, ao) > 0.0 {
			// Origin outside face ADB
			simplex.points[0] = b
			simplex.points[1] = d
			simplex.points[2] = a
			simplex.count = 3
			dir^ = adb
			return false
		}

		// Origin is inside all four tetrahedron faces: intersection confirmed
		return true
	}

	return false
}

//
// Execute GJK intersection test between two convex 3D shapes.
// Returns true if shapes intersect, false otherwise.
//
gjk_intersect :: proc "contextless" (a, b: ConvexShape) -> bool {
	// Initial search direction: vector connecting shape centers
	dir := b.transform.translation - a.transform.translation
	if vec3_length_sq(dir) < 1e-6 {
		dir = Vec3{1.0, 0.0, 0.0}
	}

	simplex := GjkSimplex{count = 0}
	p := minkowski_support(a, b, dir)
	gjk_simplex_push(&simplex, p)

	// New direction toward origin
	dir = -p

	for _ in 0 ..< GJK_MAX_ITERATIONS {
		if vec3_length_sq(dir) < 1e-8 {
			return true
		}

		p = minkowski_support(a, b, dir)
		if vec3_dot(p, dir) < 0.0 {
			// Origin cannot be reached along this direction
			return false
		}

		gjk_simplex_push(&simplex, p)
		if gjk_do_simplex(&simplex, &dir) {
			return true
		}
	}

	return false
}
