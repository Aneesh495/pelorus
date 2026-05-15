package pelorus_filter

import "core:math"
import core "../core"
import sensor "../sensor"

//
// Radius Outlier Removal (ROR) filter options.
// Prunes points that have fewer than min_neighbors within a specified search radius.
//
RadiusOutlierFilter :: struct {
	radius:         f32, // Search radius sphere (meters)
	min_neighbors:  int, // Minimum neighbor count required to retain point
	cell_size:      f32,
	hash_bins:      []int,
	bin_mask:       int,
	entry_pool:     []NeighborCell,
	owns_memory:    bool,
}

//
// Initialize RadiusOutlierFilter.
//
ror_filter_init :: proc(filter: ^RadiusOutlierFilter, radius: f32 = 0.8, min_neighbors: int = 5, capacity: int = 16384) {
	filter.radius = radius
	filter.min_neighbors = min_neighbors
	filter.cell_size = radius > 0.1 ? radius : 0.5

	table_cap := 1024
	for table_cap < capacity * 2 {
		table_cap *= 2
	}

	filter.hash_bins = make([]int, table_cap)
	filter.bin_mask = table_cap - 1
	filter.entry_pool = make([]NeighborCell, capacity)
	filter.owns_memory = true
}

//
// Release memory.
//
ror_filter_destroy :: proc(filter: ^RadiusOutlierFilter) {
	if filter.owns_memory {
		if filter.hash_bins != nil do delete(filter.hash_bins)
		if filter.entry_pool != nil do delete(filter.entry_pool)
	}
	filter.hash_bins = nil
	filter.entry_pool = nil
	filter.bin_mask = 0
}

//
// Filter PointCloud via Radius Outlier Removal, appending inliers to cloud_out.
//
filter_ror :: proc(
	filter: ^RadiusOutlierFilter,
	cloud_out: ^sensor.PointCloud,
	cloud_in: ^sensor.PointCloud,
) -> int {
	n := len(cloud_in.points)
	if n == 0 do return 0

	// 1. Build spatial hash table
	for i in 0 ..< len(filter.hash_bins) {
		filter.hash_bins[i] = -1
	}

	inv_cell := 1.0 / filter.cell_size
	entry_count := math.min(n, len(filter.entry_pool))

	for i in 0 ..< entry_count {
		p := cloud_in.points[i].position
		gx := i32(math.floor(p.x * inv_cell))
		gy := i32(math.floor(p.y * inv_cell))
		gz := i32(math.floor(p.z * inv_cell))

		key := voxel_hash_coords(gx, gy, gz)
		bin := int(key) & filter.bin_mask

		filter.entry_pool[i] = NeighborCell{
			key         = key,
			point_index = i,
			next_in_bin = filter.hash_bins[bin],
			occupied    = true,
		}
		filter.hash_bins[bin] = i
	}

	// 2. Count neighbors within search radius
	radius_sq := filter.radius * filter.radius
	min_count := filter.min_neighbors
	inliers_count := 0

	for i in 0 ..< entry_count {
		p := cloud_in.points[i].position
		gx := i32(math.floor(p.x * inv_cell))
		gy := i32(math.floor(p.y * inv_cell))
		gz := i32(math.floor(p.z * inv_cell))

		neighbor_count := 0

		// Search 3x3x3 neighboring cells
		for dx in -1 ..= 1 {
			for dy in -1 ..= 1 {
				for dz in -1 ..= 1 {
					n_key := voxel_hash_coords(gx + i32(dx), gy + i32(dy), gz + i32(dz))
					bin := int(n_key) & filter.bin_mask

					curr := filter.hash_bins[bin]
					for curr != -1 {
						cell := filter.entry_pool[curr]
						if cell.key == n_key && cell.point_index != i {
							np := cloud_in.points[cell.point_index].position
							d_sq := core.vec3_distance_sq(p, np)
							if d_sq <= radius_sq {
								neighbor_count += 1
								if neighbor_count >= min_count do break
							}
						}
						curr = cell.next_in_bin
					}
					if neighbor_count >= min_count do break
				}
				if neighbor_count >= min_count do break
			}
			if neighbor_count >= min_count do break
		}

		if neighbor_count >= min_count {
			pt := cloud_in.points[i]
			sensor.point_cloud_push(cloud_out, pt.position, pt.intensity, pt.timestamp, pt.ring, pt.normal)
			inliers_count += 1
		}
	}

	return inliers_count
}
