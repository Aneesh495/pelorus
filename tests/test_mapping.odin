package pelorus_tests

import "core:math"
import "core:testing"
import core "../src/core"
import mapping "../src/mapping"
import sensor "../src/sensor"

@(test)
test_morton_encoding :: proc(t: ^testing.T) {
	// Test positive, negative, and zero coordinates
	coords := [5][3]i32{
		{0, 0, 0},
		{10, 20, 30},
		{-15, -25, -35},
		{1024, -2048, 4096},
		{-50000, 25000, -12000},
	}

	for c in coords {
		code := mapping.morton_encode_3d(c[0], c[1], c[2])
		dec_x, dec_y, dec_z := mapping.morton_decode_3d(code)

		testing.expect_value(t, dec_x, c[0])
		testing.expect_value(t, dec_y, c[1])
		testing.expect_value(t, dec_z, c[2])
	}
}

@(test)
test_voxel_hash_table :: proc(t: ^testing.T) {
	table: mapping.VoxelHashTable
	mapping.voxel_hash_init(&table, 0.05, 100)
	defer mapping.voxel_hash_destroy(&table)

	testing.expect_value(t, table.active_blocks, 0)

	// Fetch point in block (0, 0, 0) with allocation
	node0, ok0 := mapping.voxel_hash_get_voxel(&table, core.Vec3{0.1, 0.1, 0.1}, true)
	testing.expect(t, ok0, "Voxel allocation failed")
	testing.expect(t, node0 != nil, "Node pointer nil")
	testing.expect_value(t, table.active_blocks, 1)

	// Fetch point in distant block (10, -5, 8) with allocation
	node1, ok1 := mapping.voxel_hash_get_voxel(&table, core.Vec3{5.0, -2.0, 3.5}, true)
	testing.expect(t, ok1, "Distant voxel allocation failed")
	testing.expect(t, node1 != nil, "Distant node nil")
	testing.expect_value(t, table.active_blocks, 2)

	// Fetch unallocated point without allocation -> should return false
	_, ok_missing := mapping.voxel_hash_get_voxel(&table, core.Vec3{100.0, 100.0, 100.0}, false)
	testing.expect(t, !ok_missing, "Unallocated voxel must return false")
}

@(test)
test_tsdf_ray_integration :: proc(t: ^testing.T) {
	table: mapping.VoxelHashTable
	mapping.voxel_hash_init(&table, 0.05, 500)
	defer mapping.voxel_hash_destroy(&table)

	params := mapping.tsdf_params_default(0.05)
	integrator: mapping.TsdfIntegrator
	mapping.tsdf_integrator_init(&integrator, &table, params)

	origin := core.Vec3{0.0, 0.0, 0.0}
	surface_pt := core.Vec3{2.0, 0.0, 0.0} // Surface at x = 2.0m

	// Integrate ray 5 times to build weight
	for _ in 0 ..< 5 {
		mapping.tsdf_integrate_ray(&integrator, origin, surface_pt, 1.0)
	}

	// 1. Voxel directly on surface at x = 2.0m: sdf should be ~0.0
	surf_node, ok_surf := mapping.voxel_hash_get_voxel(&table, core.Vec3{2.0, 0.0, 0.0}, false)
	testing.expect(t, ok_surf, "Surface voxel must exist")
	testing.expect(t, math.abs(surf_node.sdf) < 0.2, "Surface voxel SDF should be near zero")
	testing.expect(t, surf_node.weight >= 5.0, "Surface voxel weight should accumulate")

	// 2. Voxel in free space at x = 1.9m: sdf should be positive
	free_node, ok_free := mapping.voxel_hash_get_voxel(&table, core.Vec3{1.9, 0.0, 0.0}, false)
	testing.expect(t, ok_free, "Free space voxel must exist")
	testing.expect(t, free_node.sdf > 0.0, "Free space voxel SDF must be positive")

	// 3. Voxel behind surface at x = 2.1m: sdf should be negative
	behind_node, ok_behind := mapping.voxel_hash_get_voxel(&table, core.Vec3{2.1, 0.0, 0.0}, false)
	testing.expect(t, ok_behind, "Behind surface voxel must exist")
	testing.expect(t, behind_node.sdf < 0.0, "Behind surface voxel SDF must be negative")
}

@(test)
test_esdf_propagation :: proc(t: ^testing.T) {
	table: mapping.VoxelHashTable
	mapping.voxel_hash_init(&table, 0.1, 500)
	defer mapping.voxel_hash_destroy(&table)

	params := mapping.tsdf_params_default(0.1)
	integrator: mapping.TsdfIntegrator
	mapping.tsdf_integrator_init(&integrator, &table, params)

	// Integrate flat planar wall at z = 0.0
	origin := core.Vec3{0.0, 0.0, 3.0}
	for x in -2 ..= 2 {
		for y in -2 ..= 2 {
			surf_pt := core.Vec3{f32(x) * 0.2, f32(y) * 0.2, 0.0}
			for _ in 0 ..< 4 {
				mapping.tsdf_integrate_ray(&integrator, origin, surf_pt, 1.0)
			}
		}
	}

	esdf: mapping.EsdfMap
	mapping.esdf_map_init(&esdf, 0.1, 32, 4.0)
	defer mapping.esdf_map_destroy(&esdf)

	mapping.esdf_propagate_from_tsdf(&esdf, &table)

	// Query point at z = 1.0m directly above surface
	query_pt := core.Vec3{0.0, 0.0, 1.0}
	dist, grad, ok := mapping.esdf_query_distance_and_gradient(&esdf, query_pt)

	testing.expect(t, ok, "ESDF query must be valid within grid bounds")
	testing.expect(t, dist > 0.5 && dist <= 1.5, "ESDF distance at 1.0m above surface mismatch")
	testing.expect(t, grad.z > 0.0, "ESDF gradient should point away from obstacle surface (+Z)")
}

@(test)
test_octree_radius_search :: proc(t: ^testing.T) {
	tree: mapping.Octree
	bounds := core.aabb_make(core.Vec3{-10, -10, -10}, core.Vec3{10, 10, 10})
	mapping.octree_init(&tree, bounds, 4, 4)
	defer mapping.octree_destroy(&tree)

	points := [6]core.Vec3{
		{0.0, 0.0, 0.0},
		{0.5, 0.0, 0.0},
		{0.0, 0.5, 0.0},
		{5.0, 5.0, 5.0},
		{7.0, 7.0, 7.0},
		{-6.0, -6.0, -6.0},
	}

	mapping.octree_build(&tree, points[:])

	results := make([dynamic]int, 0, 10)
	defer delete(results)

	// Search around origin with radius 1.0m -> should find points 0, 1, 2
	mapping.octree_query_radius(&tree, 0, core.Vec3{0, 0, 0}, 1.0, &results)
	testing.expect_value(t, len(results), 3)
}

@(test)
test_raycast_line_of_sight :: proc(t: ^testing.T) {
	table: mapping.VoxelHashTable
	mapping.voxel_hash_init(&table, 0.1, 100)
	defer mapping.voxel_hash_destroy(&table)

	// Set an obstacle voxel at (1.0, 0.0, 0.0)
	node, ok := mapping.voxel_hash_get_voxel(&table, core.Vec3{1.0, 0.0, 0.0}, true)
	testing.expect(t, ok, "Obstacle voxel creation failed")
	node.sdf = -0.1 // Solid obstacle
	node.weight = 10.0
	node.flags = 3

	// Ray from (0, 0, 0) to (2, 0, 0) passes directly through the obstacle -> should be blocked
	clear_path := mapping.raycast_check_line_of_sight(&table, core.Vec3{0, 0, 0}, core.Vec3{2, 0, 0})
	testing.expect(t, !clear_path, "Ray through solid obstacle must report blocked line of sight")

	// Ray from (0, 2, 0) to (2, 2, 0) is parallel and clear -> should be clear
	clear_path2 := mapping.raycast_check_line_of_sight(&table, core.Vec3{0, 2, 0}, core.Vec3{2, 2, 0})
	testing.expect(t, clear_path2, "Clear parallel path must report line of sight")
}
