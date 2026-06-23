package pelorus_mapping

import "core:math"
import core "../core"

//
// Raycast hit result.
//
RaycastHit :: struct {
	hit:            bool,
	hit_distance:   f32,
	hit_point:      core.Vec3,
	voxel_coord:    [3]i32,
	steps_marched:  int,
}

//
// 3D Amanatides-Woo Fast Voxel Traversal (3D DDA) Algorithm.
// Traverses discrete voxel grid cells pierced by a continuous 3D ray in strict spatial order.
//
raycast_traverse_voxels :: proc(
	table: ^VoxelHashTable,
	ray: core.Ray3D,
	max_distance: f32,
	visitor: proc(table: ^VoxelHashTable, vx, vy, vz: i32, user_data: rawptr) -> bool,
	user_data: rawptr = nil,
) -> int {
	v_size := table.voxel_size
	inv_v := table.inv_voxel

	// Current continuous position and discrete voxel coordinates
	start_p := ray.origin
	vx := i32(math.floor(start_p.x * inv_v))
	vy := i32(math.floor(start_p.y * inv_v))
	vz := i32(math.floor(start_p.z * inv_v))

	dir := ray.direction

	step_x: i32 = dir.x >= 0.0 ? 1 : -1
	step_y: i32 = dir.y >= 0.0 ? 1 : -1
	step_z: i32 = dir.z >= 0.0 ? 1 : -1

	// Distance along ray to cross next voxel boundary
	t_max_x: f32 = 1e30
	t_max_y: f32 = 1e30
	t_max_z: f32 = 1e30

	t_delta_x: f32 = 1e30
	t_delta_y: f32 = 1e30
	t_delta_z: f32 = 1e30

	if math.abs(dir.x) > 1e-6 {
		voxel_boundary_x := (f32(vx + (step_x > 0 ? 1 : 0))) * v_size
		t_max_x = (voxel_boundary_x - start_p.x) / dir.x
		t_delta_x = v_size / math.abs(dir.x)
	}

	if math.abs(dir.y) > 1e-6 {
		voxel_boundary_y := (f32(vy + (step_y > 0 ? 1 : 0))) * v_size
		t_max_y = (voxel_boundary_y - start_p.y) / dir.y
		t_delta_y = v_size / math.abs(dir.y)
	}

	if math.abs(dir.z) > 1e-6 {
		voxel_boundary_z := (f32(vz + (step_z > 0 ? 1 : 0))) * v_size
		t_max_z = (voxel_boundary_z - start_p.z) / dir.z
		t_delta_z = v_size / math.abs(dir.z)
	}

	t: f32 = 0.0
	steps := 0

	for t <= max_distance {
		// Invoke visitor callback on current voxel
		stop := visitor(table, vx, vy, vz, user_data)
		steps += 1
		if stop do break

		// Advance to next voxel along smallest t_max
		if t_max_x < t_max_y {
			if t_max_x < t_max_z {
				t = t_max_x
				t_max_x += t_delta_x
				vx += step_x
			} else {
				t = t_max_z
				t_max_z += t_delta_z
				vz += step_z
			}
		} else {
			if t_max_y < t_max_z {
				t = t_max_y
				t_max_y += t_delta_y
				vy += step_y
			} else {
				t = t_max_z
				t_max_z += t_delta_z
				vz += step_z
			}
		}
	}

	return steps
}

//
// Fast line-of-sight visibility check between two 3D points.
// Returns true if straight path between p1 and p2 is free of solid obstacles.
//
raycast_check_line_of_sight :: proc(
	table: ^VoxelHashTable,
	p1, p2: core.Vec3,
) -> bool {
	dir := p2 - p1
	dist := core.vec3_length(dir)
	if dist < 1e-4 do return true

	ray := core.ray3d_make(p1, dir)

	CallbackData :: struct {
		blocked: bool,
	}

	cb_data := CallbackData{blocked = false}

	vis_proc :: proc(tbl: ^VoxelHashTable, vx, vy, vz: i32, user_data: rawptr) -> bool {
		data := (^CallbackData)(user_data)
		bx := vx >> 3
		by := vy >> 3
		bz := vz >> 3
		block := voxel_hash_find_block(tbl, bx, by, bz)
		if block != nil {
			lx := vx & 7
			ly := vy & 7
			lz := vz & 7
			idx := voxel_local_index(lx, ly, lz)
			node := block.voxels[idx]
			// If voxel is observed and solid surface/interior (sdf <= 0)
			if (node.flags & 1) != 0 && node.weight > 1.0 && node.sdf <= 0.05 {
				data.blocked = true
				return true // Stop traversal
			}
		}
		return false
	}

	raycast_traverse_voxels(table, ray, dist, vis_proc, &cb_data)
	return !cb_data.blocked
}
