package pelorus_mapping

import "core:math"
import core "../core"
import sensor "../sensor"

//
// Configuration parameters for Truncated Signed Distance Field (TSDF) integration.
//
TsdfParams :: struct {
	truncation_distance: f32, // Truncation band half-width delta (meters, e.g. 0.15m)
	max_weight:          f32, // Maximum accumulated observation weight (e.g. 100.0)
	ray_step_factor:     f32, // Ray marching step size relative to voxel size
	min_ray_range:       f32, // Minimum valid range to integrate
	max_ray_range:       f32, // Maximum valid range to integrate
}

//
// Default TSDF integration parameters.
//
tsdf_params_default :: proc "contextless" (voxel_size: f32 = 0.05) -> TsdfParams {
	return TsdfParams{
		truncation_distance = voxel_size * 3.0,
		max_weight          = 100.0,
		ray_step_factor     = 0.8,
		min_ray_range       = 0.5,
		max_ray_range       = 50.0,
	}
}

//
// Volumetric TSDF Integrator.
// Updates sparse voxel grid from 3D sensor scans via raycasted projective distance updates.
//
TsdfIntegrator :: struct {
	params: TsdfParams,
	table:  ^VoxelHashTable,
}

//
// Initialize TSDF integrator wrapping an existing VoxelHashTable.
//
tsdf_integrator_init :: proc(integrator: ^TsdfIntegrator, table: ^VoxelHashTable, params: TsdfParams) {
	integrator.table = table
	integrator.params = params
}

//
// Integrate a single laser ray into the TSDF grid from sensor origin to measurement point.
//
tsdf_integrate_ray :: proc(
	integrator: ^TsdfIntegrator,
	origin: core.Vec3,
	point: core.Vec3,
	weight: f32 = 1.0,
) {
	ray_dir := point - origin
	ray_length := core.vec3_length(ray_dir)

	if ray_length < integrator.params.min_ray_range || ray_length > integrator.params.max_ray_range {
		return
	}

	norm_dir := ray_dir * (1.0 / ray_length)
	trunc := integrator.params.truncation_distance
	step_size := integrator.table.voxel_size * integrator.params.ray_step_factor

	// Ray march from (ray_length - trunc) to (ray_length + trunc)
	// Focuses updates strictly within the truncation band around the surface
	start_dist := math.max(0.1, ray_length - trunc)
	end_dist := ray_length + trunc

	d := start_dist
	for d <= end_dist {
		sample_pos := origin + norm_dir * d

		// Projective signed distance: eta = ray_length - d
		eta := ray_length - d

		// Normalized truncated SDF in [-1, +1]
		normalized_sdf := math.clamp(eta / trunc, -1.0, 1.0)

		node, ok := voxel_hash_get_voxel(integrator.table, sample_pos, true)
		if ok {
			// Running weighted average update
			old_w := node.weight
			new_w := math.min(integrator.params.max_weight, old_w + weight)

			if (old_w + weight) > 0.0 {
				node.sdf = (old_w * node.sdf + weight * normalized_sdf) / (old_w + weight)
			}
			node.weight = new_w
			node.flags |= 1 // Mark observed

			// Flag surface boundary if close to zero crossing
			if math.abs(node.sdf) < 0.2 && node.weight > 2.0 {
				node.flags |= 2
			}
		}

		d += step_size
	}
}

//
// Integrate full PointCloud into TSDF voxel hash table.
//
tsdf_integrate_point_cloud :: proc(
	integrator: ^TsdfIntegrator,
	cloud: ^sensor.PointCloud,
	sensor_origin: core.Vec3,
) {
	for pt in cloud.points {
		tsdf_integrate_ray(integrator, sensor_origin, pt.position, 1.0)
	}
}

//
// Extract zero-crossing surface point cloud from the TSDF voxel grid.
//
tsdf_extract_surface_points :: proc(
	table: ^VoxelHashTable,
	cloud_out: ^sensor.PointCloud,
	weight_threshold: f32 = 2.0,
) -> int {
	extracted := 0
	v_size := table.voxel_size

	for block in table.blocks {
		base_vx := block.coord_x << 3
		base_vy := block.coord_y << 3
		base_vz := block.coord_z << 3

		for lz in 0 ..< VOXEL_BLOCK_SIZE {
			for ly in 0 ..< VOXEL_BLOCK_SIZE {
				for lx in 0 ..< VOXEL_BLOCK_SIZE {
					idx := voxel_local_index(i32(lx), i32(ly), i32(lz))
					node := block.voxels[idx]

					// Check if voxel is close to zero-crossing with sufficient confidence
					if (node.flags & 1) != 0 && node.weight >= weight_threshold && math.abs(node.sdf) < 0.25 {
						world_x := (f32(base_vx + i32(lx)) + 0.5) * v_size
						world_y := (f32(base_vy + i32(ly)) + 0.5) * v_size
						world_z := (f32(base_vz + i32(lz)) + 0.5) * v_size

						pos := core.Vec3{world_x, world_y, world_z}
						intensity := (1.0 - math.abs(node.sdf)) * 255.0
						sensor.point_cloud_push(cloud_out, pos, intensity)
						extracted += 1
					}
				}
			}
		}
	}

	return extracted
}
