package pelorus_filter

import "core:math"
import core "../core"
import sensor "../sensor"

//
// Internal voxel aggregation cell.
//
VoxelCell :: struct {
	key:          u64,
	sum_x:        f64,
	sum_y:        f64,
	sum_z:        f64,
	sum_intensity: f32,
	point_count:  u32,
	min_time:     f64,
	ring:         u16,
	occupied:     bool,
}

//
// 3D Voxel Grid Downsampling Filter with geometric centroid aggregation.
// Partitions 3D space into discrete cubic grid cells, replacing all points within each
// voxel by their exact spatial centroid and mean intensity.
//
VoxelGridFilter :: struct {
	leaf_size:             core.Vec3, // Dimensions (dx, dy, dz) of individual voxel cells
	inv_leaf:              core.Vec3, // Precomputed 1.0 / leaf_size
	min_points_per_voxel:  u32,       // Prunes isolated sparse noise
	hash_table:            []VoxelCell,
	table_mask:            u64,
	owns_memory:           bool,
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
// Initialize VoxelGridFilter with leaf size and expected point capacity.
//
voxel_grid_init :: proc(filter: ^VoxelGridFilter, leaf_size: core.Vec3, capacity: int = 16384) {
	filter.leaf_size = leaf_size
	filter.inv_leaf = core.Vec3{
		1.0 / (leaf_size.x > 1e-4 ? leaf_size.x : 0.1),
		1.0 / (leaf_size.y > 1e-4 ? leaf_size.y : 0.1),
		1.0 / (leaf_size.z > 1e-4 ? leaf_size.z : 0.1),
	}
	filter.min_points_per_voxel = 1

	// Round capacity to power of two for fast bitwise table masking
	table_cap := 1024
	for table_cap < capacity * 2 {
		table_cap *= 2
	}

	filter.hash_table = make([]VoxelCell, table_cap)
	filter.table_mask = u64(table_cap - 1)
	filter.owns_memory = true
}

//
// Destroy filter and release hash table memory.
//
voxel_grid_destroy :: proc(filter: ^VoxelGridFilter) {
	if filter.owns_memory && filter.hash_table != nil {
		delete(filter.hash_table)
	}
	filter.hash_table = nil
	filter.table_mask = 0
}

//
// Downsample input PointCloud, appending aggregated centroid points to cloud_out.
// Returns count of output points generated.
//
voxel_grid_filter :: proc(
	filter: ^VoxelGridFilter,
	cloud_out: ^sensor.PointCloud,
	cloud_in: ^sensor.PointCloud,
) -> int {
	n := len(cloud_in.points)
	if n == 0 do return 0

	// Clear hash table
	for i in 0 ..< len(filter.hash_table) {
		filter.hash_table[i].occupied = false
	}

	mask := filter.table_mask
	inv := filter.inv_leaf

	// 1. Accumulate points into voxel cells
	for pt in cloud_in.points {
		gx := i32(math.floor(pt.position.x * inv.x))
		gy := i32(math.floor(pt.position.y * inv.y))
		gz := i32(math.floor(pt.position.z * inv.z))

		key := voxel_hash_coords(gx, gy, gz)
		slot := key & mask

		// Linear probing
		for {
			cell := &filter.hash_table[slot]
			if !cell.occupied {
				// Empty slot found, initialize new cell
				cell.key = key
				cell.sum_x = f64(pt.position.x)
				cell.sum_y = f64(pt.position.y)
				cell.sum_z = f64(pt.position.z)
				cell.sum_intensity = pt.intensity
				cell.point_count = 1
				cell.min_time = pt.timestamp
				cell.ring = pt.ring
				cell.occupied = true
				break
			}

			if cell.key == key {
				// Match existing cell, accumulate
				cell.sum_x += f64(pt.position.x)
				cell.sum_y += f64(pt.position.y)
				cell.sum_z += f64(pt.position.z)
				cell.sum_intensity += pt.intensity
				cell.point_count += 1
				if pt.timestamp < cell.min_time {
					cell.min_time = pt.timestamp
				}
				break
			}

			slot = (slot + 1) & mask
		}
	}

	// 2. Extract centroids from occupied cells satisfying minimum point threshold
	output_count := 0
	for cell in filter.hash_table {
		if cell.occupied && cell.point_count >= filter.min_points_per_voxel {
			inv_cnt := 1.0 / f64(cell.point_count)
			centroid := core.Vec3{
				f32(cell.sum_x * inv_cnt),
				f32(cell.sum_y * inv_cnt),
				f32(cell.sum_z * inv_cnt),
			}
			mean_intensity := cell.sum_intensity / f32(cell.point_count)

			sensor.point_cloud_push(cloud_out, centroid, mean_intensity, cell.min_time, cell.ring)
			output_count += 1
		}
	}

	return output_count
}
