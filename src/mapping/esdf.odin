package pelorus_mapping

import "core:math"
import core "../core"

//
// Individual voxel in Euclidean Signed Distance Field (ESDF).
// Stores exact Euclidean metric distance to nearest obstacle surface.
//
EsdfVoxel :: struct {
	distance:   f32, // Metric Euclidean distance to obstacle (meters, negative inside)
	parent_vx:  i32, // Coordinate of closest surface obstacle voxel
	parent_vy:  i32,
	parent_vz:  i32,
	observed:   bool,
	fixed:      bool,
}

//
// 3D Euclidean Signed Distance Field (ESDF) Map.
// Propagates full-workspace obstacle clearance distances and spatial gradients for collision planning.
//
EsdfMap :: struct {
	voxel_size:   f32,
	inv_voxel:    f32,
	max_distance: f32, // Maximum clearance distance to propagate (e.g. 5.0m)
	grid_dim:     i32, // Bounded working volume dimension
	voxels:       []EsdfVoxel,
	origin:       core.Vec3,
	owns_memory:  bool,
}

//
// Initialize ESDF Map over bounded 3D grid volume.
//
esdf_map_init :: proc(map_obj: ^EsdfMap, voxel_size: f32 = 0.1, grid_dim: i32 = 64, max_distance: f32 = 5.0) {
	map_obj.voxel_size = voxel_size > 0.01 ? voxel_size : 0.1
	map_obj.inv_voxel = 1.0 / map_obj.voxel_size
	map_obj.max_distance = max_distance
	map_obj.grid_dim = grid_dim

	total_voxels := int(grid_dim * grid_dim * grid_dim)
	map_obj.voxels = make([]EsdfVoxel, total_voxels)
	map_obj.origin = core.Vec3{
		-f32(grid_dim) * 0.5 * map_obj.voxel_size,
		-f32(grid_dim) * 0.5 * map_obj.voxel_size,
		-f32(grid_dim) * 0.5 * map_obj.voxel_size,
	}
	map_obj.owns_memory = true

	for i in 0 ..< total_voxels {
		map_obj.voxels[i] = EsdfVoxel{
			distance  = max_distance,
			parent_vx = 0,
			parent_vy = 0,
			parent_vz = 0,
			observed  = false,
			fixed     = false,
		}
	}
}

//
// Destroy ESDF map.
//
esdf_map_destroy :: proc(map_obj: ^EsdfMap) {
	if map_obj.owns_memory && map_obj.voxels != nil {
		delete(map_obj.voxels)
	}
	map_obj.voxels = nil
}

//
// Compute internal linear voxel array index from 3D grid coordinates.
//
esdf_linear_index :: proc "contextless" (map_obj: ^EsdfMap, gx, gy, gz: i32) -> (int, bool) {
	dim := map_obj.grid_dim
	if gx < 0 || gx >= dim || gy < 0 || gy >= dim || gz < 0 || gz >= dim {
		return 0, false
	}
	return int(gz * dim * dim + gy * dim + gx), true
}

//
// Multi-source BFS Wavefront Propagation: initializes surface voxels from TSDF
// and marches clearance distances outward into empty space.
//
esdf_propagate_from_tsdf :: proc(
	esdf: ^EsdfMap,
	tsdf_table: ^VoxelHashTable,
) {
	dim := esdf.grid_dim
	total_voxels := int(dim * dim * dim)
	v_size := esdf.voxel_size
	max_d := esdf.max_distance

	// 1. Reset all voxels to max distance
	for i in 0 ..< total_voxels {
		esdf.voxels[i].distance = max_d
		esdf.voxels[i].observed = false
		esdf.voxels[i].fixed = false
	}

	queue := make([dynamic]i32, 0, 4096)
	defer delete(queue)

	// 2. Extract zero-crossing surface seeds from TSDF
	for gz in 0 ..< dim {
		for gy in 0 ..< dim {
			for gx in 0 ..< dim {
				world_pos := esdf.origin + core.Vec3{
					(f32(gx) + 0.5) * v_size,
					(f32(gy) + 0.5) * v_size,
					(f32(gz) + 0.5) * v_size,
				}

				node, ok := voxel_hash_get_voxel(tsdf_table, world_pos, false)
				if ok && (node.flags & 1) != 0 {
					idx, _ := esdf_linear_index(esdf, gx, gy, gz)
					esdf.voxels[idx].observed = true

					// If node is an obstacle surface voxel (near zero-crossing or solid)
					if math.abs(node.sdf) < 0.3 && node.weight > 1.0 {
						esdf.voxels[idx].distance = 0.0
						esdf.voxels[idx].fixed = true
						esdf.voxels[idx].parent_vx = gx
						esdf.voxels[idx].parent_vy = gy
						esdf.voxels[idx].parent_vz = gz

						append(&queue, gx)
						append(&queue, gy)
						append(&queue, gz)
					}
				}
			}
		}
	}

	// 3. Wavefront propagation (6-connected neighbors)
	offsets := [6][3]i32{
		{ 1,  0,  0}, {-1,  0,  0},
		{ 0,  1,  0}, { 0, -1,  0},
		{ 0,  0,  1}, { 0,  0, -1},
	}

	q_head := 0
	for q_head < len(queue) {
		cx := queue[q_head]
		cy := queue[q_head + 1]
		cz := queue[q_head + 2]
		q_head += 3

		curr_idx, _ := esdf_linear_index(esdf, cx, cy, cz)
		parent_x := esdf.voxels[curr_idx].parent_vx
		parent_y := esdf.voxels[curr_idx].parent_vy
		parent_z := esdf.voxels[curr_idx].parent_vz

		for off in offsets {
			nx := cx + off[0]
			ny := cy + off[1]
			nz := cz + off[2]

			n_idx, ok_n := esdf_linear_index(esdf, nx, ny, nz)
			if !ok_n do continue

			if esdf.voxels[n_idx].fixed do continue

			// Exact Euclidean distance from parent seed voxel
			dx := f32(nx - parent_x) * v_size
			dy := f32(ny - parent_y) * v_size
			dz := f32(nz - parent_z) * v_size
			dist := math.sqrt(dx * dx + dy * dy + dz * dz)

			if dist < esdf.voxels[n_idx].distance && dist <= max_d {
				esdf.voxels[n_idx].distance = dist
				esdf.voxels[n_idx].parent_vx = parent_x
				esdf.voxels[n_idx].parent_vy = parent_y
				esdf.voxels[n_idx].parent_vz = parent_z

				append(&queue, nx)
				append(&queue, ny)
				append(&queue, nz)
			}
		}
	}
}

//
// Query continuous distance and spatial gradient at an arbitrary world position p.
// Returns distance (meters), gradient vector, and validity flag.
//
esdf_query_distance_and_gradient :: proc(
	esdf: ^EsdfMap,
	p: core.Vec3,
) -> (distance: f32, gradient: core.Vec3, valid: bool) {
	rel := p - esdf.origin
	gx := i32(math.floor(rel.x * esdf.inv_voxel))
	gy := i32(math.floor(rel.y * esdf.inv_voxel))
	gz := i32(math.floor(rel.z * esdf.inv_voxel))

	idx, ok := esdf_linear_index(esdf, gx, gy, gz)
	if !ok {
		return esdf.max_distance, core.vec3_zero(), false
	}

	dist := esdf.voxels[idx].distance

	// Central finite differences for gradient: grad_x = (d(x+1) - d(x-1)) / (2 * h)
	idx_xp, ok_xp := esdf_linear_index(esdf, gx + 1, gy, gz)
	idx_xm, ok_xm := esdf_linear_index(esdf, gx - 1, gy, gz)
	idx_yp, ok_yp := esdf_linear_index(esdf, gx, gy + 1, gz)
	idx_ym, ok_ym := esdf_linear_index(esdf, gx, gy - 1, gz)
	idx_zp, ok_zp := esdf_linear_index(esdf, gx, gy, gz + 1)
	idx_zm, ok_zm := esdf_linear_index(esdf, gx, gy, gz - 1)

	d_xp := ok_xp ? esdf.voxels[idx_xp].distance : dist
	d_xm := ok_xm ? esdf.voxels[idx_xm].distance : dist
	d_yp := ok_yp ? esdf.voxels[idx_yp].distance : dist
	d_ym := ok_ym ? esdf.voxels[idx_ym].distance : dist
	d_zp := ok_zp ? esdf.voxels[idx_zp].distance : dist
	d_zm := ok_zm ? esdf.voxels[idx_zm].distance : dist

	inv_2h := 0.5 * esdf.inv_voxel
	grad := core.Vec3{
		(d_xp - d_xm) * inv_2h,
		(d_yp - d_ym) * inv_2h,
		(d_zp - d_zm) * inv_2h,
	}

	return dist, grad, true
}
