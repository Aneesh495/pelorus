package pelorus_filter

import "core:math"
import core "../core"
import sensor "../sensor"

//
// Spatial hash cell for local neighborhood queries.
//
NeighborCell :: struct {
	key:         u64,
	point_index: int,
	next_in_bin: int,
	occupied:    bool,
}

//
// Statistical Outlier Removal (SOR) filter.
// Computes mean distance to k-nearest neighbors for each point, calculating the global
// mean mu and standard deviation sigma. Points with mean distance > mu + alpha * sigma are rejected.
//
StatisticalOutlierFilter :: struct {
	mean_k:      int, // Number of nearest neighbors to evaluate (e.g. 10 to 30)
	std_mul:     f32, // Standard deviation multiplier threshold alpha (e.g. 1.0 to 2.5)
	cell_size:   f32, // Spatial hashing grid cell size
	hash_bins:   []int,
	bin_mask:    int,
	entry_pool:  []NeighborCell,
	owns_memory: bool,
}

//
// Initialize StatisticalOutlierFilter with neighborhood size and capacity.
//
sor_filter_init :: proc(filter: ^StatisticalOutlierFilter, mean_k: int = 16, std_mul: f32 = 1.5, capacity: int = 16384) {
	filter.mean_k = mean_k
	filter.std_mul = std_mul
	filter.cell_size = 0.5

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
sor_filter_destroy :: proc(filter: ^StatisticalOutlierFilter) {
	if filter.owns_memory {
		if filter.hash_bins != nil do delete(filter.hash_bins)
		if filter.entry_pool != nil do delete(filter.entry_pool)
	}
	filter.hash_bins = nil
	filter.entry_pool = nil
	filter.bin_mask = 0
}

//
// Filter PointCloud via Statistical Outlier Removal.
//
filter_sor :: proc(
	filter: ^StatisticalOutlierFilter,
	cloud_out: ^sensor.PointCloud,
	cloud_in: ^sensor.PointCloud,
) -> int {
	n := len(cloud_in.points)
	if n == 0 do return 0
	if n <= filter.mean_k {
		// Too few points for statistical thresholding, retain all
		for pt in cloud_in.points {
			sensor.point_cloud_push(cloud_out, pt.position, pt.intensity, pt.timestamp, pt.ring, pt.normal)
		}
		return n
	}

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

	// 2. Compute mean distance to k neighbors for each point
	mean_distances := make([]f32, n)
	defer delete(mean_distances)

	var_sum: f64 = 0.0
	k := filter.mean_k

	for i in 0 ..< entry_count {
		p := cloud_in.points[i].position
		gx := i32(math.floor(p.x * inv_cell))
		gy := i32(math.floor(p.y * inv_cell))
		gz := i32(math.floor(p.z * inv_cell))

		dist_acc: f32 = 0.0
		neighbors_found := 0

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
							d := core.vec3_distance(p, np)
							dist_acc += d
							neighbors_found += 1
							if neighbors_found >= k do break
						}
						curr = cell.next_in_bin
					}
					if neighbors_found >= k do break
				}
				if neighbors_found >= k do break
			}
			if neighbors_found >= k do break
		}

		mean_d := neighbors_found > 0 ? (dist_acc / f32(neighbors_found)) : 10.0
		mean_distances[i] = mean_d
		var_sum += f64(mean_d)
	}

	// 3. Compute global mean and standard deviation
	global_mean := f32(var_sum / f64(entry_count))

	var_sq_sum: f64 = 0.0
	for i in 0 ..< entry_count {
		diff := mean_distances[i] - global_mean
		var_sq_sum += f64(diff * diff)
	}
	global_std := math.sqrt(f32(var_sq_sum / f64(entry_count)))

	// Threshold: d_max = mu + alpha * sigma
	threshold := global_mean + filter.std_mul * global_std

	// 4. Retain inliers
	inliers_count := 0
	for i in 0 ..< entry_count {
		if mean_distances[i] <= threshold {
			pt := cloud_in.points[i]
			sensor.point_cloud_push(cloud_out, pt.position, pt.intensity, pt.timestamp, pt.ring, pt.normal)
			inliers_count += 1
		}
	}

	return inliers_count
}
