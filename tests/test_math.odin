package pelorus_tests

import "core:math"
import "core:testing"
import core "../src/core"

@(test)
test_vector_operations :: proc(t: ^testing.T) {
	v1 := core.Vec3{1.0, 2.0, 3.0}
	v2 := core.Vec3{4.0, 5.0, 6.0}

	dot := core.vec3_dot(v1, v2)
	testing.expect_value(t, dot, 32.0)

	cross := core.vec3_cross(v1, v2)
	expected_cross := core.Vec3{-3.0, 6.0, -3.0}
	testing.expect(t, core.vec3_distance(cross, expected_cross) < 1e-5, "Cross product mismatch")

	norm := core.vec3_normalize(v1)
	testing.expect(t, math.abs(core.vec3_length(norm) - 1.0) < 1e-5, "Normalized length must be 1.0")
}

@(test)
test_quaternion_operations :: proc(t: ^testing.T) {
	q_id := core.quat_identity()
	testing.expect_value(t, q_id.w, 1.0)

	axis := core.Vec3{0.0, 0.0, 1.0}
	q_rot := core.quat_from_axis_angle(axis, f32(math.PI * 0.5))

	p := core.Vec3{1.0, 0.0, 0.0}
	p_rotated := core.quat_rotate_vec3(q_rot, p)
	expected_rotated := core.Vec3{0.0, 1.0, 0.0}
	testing.expect(t, core.vec3_distance(p_rotated, expected_rotated) < 1e-5, "90-deg Z rotation failed")

	q_inv := core.quat_inverse(q_rot)
	p_restored := core.quat_rotate_vec3(q_inv, p_rotated)
	testing.expect(t, core.vec3_distance(p_restored, p) < 1e-5, "Quaternion inverse restoration failed")

	q_half := core.quat_slerp(q_id, q_rot, 0.5)
	p_half := core.quat_rotate_vec3(q_half, p)
	expected_half := core.Vec3{math.cos(f32(math.PI * 0.25)), math.sin(f32(math.PI * 0.25)), 0.0}
	testing.expect(t, core.vec3_distance(p_half, expected_half) < 1e-5, "Slerp interpolation failed")
}

@(test)
test_so3_lie_algebra :: proc(t: ^testing.T) {
	phi := core.Vec3{0.1, -0.2, 0.3}

	// Test hat and vee isomorphism
	hat := core.so3_hat(phi)
	recovered_phi := core.so3_vee(hat)
	testing.expect(t, core.vec3_distance(phi, recovered_phi) < 1e-6, "Hat/Vee isomorphism failed")

	// Test exp and log roundtrip
	rot_mat := core.so3_exp_mat3(phi)
	log_phi := core.so3_log_mat3(rot_mat)
	testing.expect(t, core.vec3_distance(phi, log_phi) < 1e-4, "SO(3) exp/log roundtrip failed")

	// Test quaternion exp and log roundtrip
	q := core.so3_exp_quat(phi)
	log_q := core.so3_log_quat(q)
	testing.expect(t, core.vec3_distance(phi, log_q) < 1e-4, "SO(3) quaternion exp/log roundtrip failed")
}

@(test)
test_se3_rigid_transforms :: proc(t: ^testing.T) {
	xi := core.Twist6{
		rho = core.Vec3{1.5, -2.0, 0.5},
		phi = core.Vec3{0.05, 0.1, -0.15},
	}

	tf := core.se3_exp(xi)
	recovered_xi := core.se3_log(tf)

	testing.expect(t, core.vec3_distance(xi.rho, recovered_xi.rho) < 1e-4, "SE(3) rho exp/log failed")
	testing.expect(t, core.vec3_distance(xi.phi, recovered_xi.phi) < 1e-4, "SE(3) phi exp/log failed")

	// Test Adjoint consistency: Ad_T * xi
	adj := core.se3_adjoint(tf)
	testing.expect(t, math.abs(adj[0, 0]) > 0.0, "Adjoint matrix block missing")

	adj_applied := core.se3_adjoint_apply(tf, xi)
	testing.expect(t, core.vec3_length(adj_applied.phi) > 0.0, "Adjoint apply rotation failed")
}

@(test)
test_se3_geodesic_interpolation :: proc(t: ^testing.T) {
	t0 := core.Transform3D{
		translation = core.Vec3{0.0, 0.0, 0.0},
		rotation    = core.quat_identity(),
	}
	t1 := core.Transform3D{
		translation = core.Vec3{10.0, 20.0, 30.0},
		rotation    = core.quat_from_axis_angle(core.Vec3{0, 0, 1}, f32(math.PI * 0.5)),
	}

	// At tau = 0, should match t0
	interp0 := core.se3_geodesic_interpolate(t0, t1, 0.0)
	testing.expect(t, core.vec3_distance(interp0.translation, t0.translation) < 1e-5, "Tau=0 interpolation failed")

	// At tau = 1, should match t1
	interp1 := core.se3_geodesic_interpolate(t0, t1, 1.0)
	testing.expect(t, core.vec3_distance(interp1.translation, t1.translation) < 1e-5, "Tau=1 interpolation failed")

	// Pure translation midpoint test
	t1_trans := core.Transform3D{
		translation = core.Vec3{10.0, 20.0, 30.0},
		rotation    = core.quat_identity(),
	}
	interp_half := core.se3_geodesic_interpolate(t0, t1_trans, 0.5)
	expected_trans := core.Vec3{5.0, 10.0, 15.0}
	testing.expect(t, core.vec3_distance(interp_half.translation, expected_trans) < 1e-4, "Midpoint pure translation failed")

	// Screw motion with rotation: Z is along rotation axis so Z midpoint must be 15
	interp_screw := core.se3_geodesic_interpolate(t0, t1, 0.5)
	testing.expect(t, math.abs(interp_screw.translation.z - 15.0) < 1e-4, "Screw motion Z translation failed")
}

@(test)
test_geometry_and_gjk :: proc(t: ^testing.T) {
	box_a := core.aabb_make(core.Vec3{-1, -1, -1}, core.Vec3{1, 1, 1})
	box_b := core.aabb_make(core.Vec3{0.5, 0.5, 0.5}, core.Vec3{2, 2, 2})
	box_c := core.aabb_make(core.Vec3{3, 3, 3}, core.Vec3{5, 5, 5})

	testing.expect(t, core.aabb_intersects_aabb(box_a, box_b), "Overlapping AABB should intersect")
	testing.expect(t, !core.aabb_intersects_aabb(box_a, box_c), "Separated AABB should not intersect")

	ray := core.ray3d_make(core.Vec3{-5, 0, 0}, core.Vec3{1, 0, 0})
	hit, t_entry, t_exit := core.aabb_intersects_ray(box_a, ray)
	testing.expect(t, hit, "Ray should hit box_a")
	testing.expect(t, math.abs(t_entry - 4.0) < 1e-4, "Ray entry distance mismatch")
	testing.expect(t, math.abs(t_exit - 6.0) < 1e-4, "Ray exit distance mismatch")

	// GJK convex collision test
	shape1 := core.ConvexShape{
		kind      = .Sphere,
		transform = core.Transform3D{translation = core.Vec3{0, 0, 0}, rotation = core.quat_identity()},
		sphere    = {radius = 1.0},
	}
	shape2 := core.ConvexShape{
		kind      = .Sphere,
		transform = core.Transform3D{translation = core.Vec3{1.5, 0, 0}, rotation = core.quat_identity()},
		sphere    = {radius = 1.0},
	}
	shape3 := core.ConvexShape{
		kind      = .Sphere,
		transform = core.Transform3D{translation = core.Vec3{3.0, 0, 0}, rotation = core.quat_identity()},
		sphere    = {radius = 0.5},
	}

	testing.expect(t, core.gjk_intersect(shape1, shape2), "Overlapping spheres must intersect in GJK")
	testing.expect(t, !core.gjk_intersect(shape1, shape3), "Separated spheres must not intersect in GJK")
}

@(test)
test_statistics_and_cholesky :: proc(t: ^testing.T) {
	// Construct symmetric positive definite 3x3 matrix
	cov := core.Mat3{
		4.0, 1.0, 0.5,
		1.0, 3.0, 0.2,
		0.5, 0.2, 2.0,
	}

	l, ok := core.cholesky_3x3(cov)
	testing.expect(t, ok, "Cholesky decomposition must succeed")

	// Verify L * L^T == cov
	reconstructed := l * core.Mat3{
		l[0, 0], l[1, 0], l[2, 0],
		l[0, 1], l[1, 1], l[2, 1],
		l[0, 2], l[1, 2], l[2, 2],
	}

	for r in 0 ..< 3 {
		for c in 0 ..< 3 {
			testing.expect(t, math.abs(cov[r, c] - reconstructed[r, c]) < 1e-4, "L*L^T mismatch")
		}
	}

	// Mahalanobis distance test
	mu := core.Vec3{0, 0, 0}
	x := core.Vec3{1, 1, 1}
	d_sq := core.mahalanobis_distance_sq_3d(x, mu, cov)
	testing.expect(t, d_sq > 0.0, "Mahalanobis distance squared must be positive")
}
