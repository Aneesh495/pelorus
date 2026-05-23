package pelorus_filter

import "core:math"
import core "../core"
import sensor "../sensor"

//
// Normal and curvature estimation result for a local neighborhood.
//
NormalResult :: struct {
	normal:    core.Vec3,
	curvature: f32, // Surface curvature score in [0, 1/3]
	valid:     bool,
}

//
// Surface Normal Estimator using Principal Component Analysis (PCA) over k-neighborhoods.
// Computes local tangent plane normals and curvature, ensuring viewpoint orientation consistency.
//
NormalEstimator :: struct {
	k_neighbors:   int,       // Number of local neighbors (typically 10 to 30)
	search_radius: f32,       // Maximum search radius (meters)
	viewpoint:     core.Vec3, // Sensor viewpoint coordinate for normal flipping
	cell_size:     f32,
	hash_bins:     []int,
	bin_mask:      int,
	entry_pool:    []NeighborCell,
	owns_memory:   bool,
}

//
// Initialize NormalEstimator with neighborhood parameters.
//
normal_estimator_init :: proc(
	est: ^NormalEstimator,
	k_neighbors: int = 15,
	search_radius: f32 = 0.5,
	viewpoint: core.Vec3 = core.Vec3{0, 0, 0},
	capacity: int = 16384,
) {
	est.k_neighbors = k_neighbors
	est.search_radius = search_radius
	est.viewpoint = viewpoint
	est.cell_size = search_radius > 0.1 ? search_radius : 0.5

	table_cap := 1024
	for table_cap < capacity * 2 {
		table_cap *= 2
	}

	est.hash_bins = make([]int, table_cap)
	est.bin_mask = table_cap - 1
	est.entry_pool = make([]NeighborCell, capacity)
	est.owns_memory = true
}

//
// Release memory.
//
normal_estimator_destroy :: proc(est: ^NormalEstimator) {
	if est.owns_memory {
		if est.hash_bins != nil do delete(est.hash_bins)
		if est.entry_pool != nil do delete(est.entry_pool)
	}
	est.hash_bins = nil
	est.entry_pool = nil
	est.bin_mask = 0
}

//
// Estimate surface normal vector from a cluster of neighboring points via PCA covariance decomposition.
//
estimate_normal_pca :: proc "contextless" (points: []core.Vec3, viewpoint: core.Vec3) -> NormalResult {
	n := len(points)
	if n < 3 {
		return NormalResult{normal = core.Vec3{0, 0, 1}, curvature = 0.0, valid = false}
	}

	mean, cov := core.compute_mean_covariance_3d(points)
	eigenvalues, eigenvectors := core.eigen_decomposition_3x3(cov)

	// In eigen_decomposition_3x3, eigenvalues are sorted descending: lambda.x >= lambda.y >= lambda.z
	// Smallest eigenvalue is in lambda.z, with corresponding eigenvector in column 2 of V
	normal := core.Vec3{
		eigenvectors[0, 2],
		eigenvectors[1, 2],
		eigenvectors[2, 2],
	}

	norm_len := core.vec3_length(normal)
	if norm_len < 1e-5 {
		normal = core.Vec3{0, 0, 1}
	} else {
		normal = normal * (1.0 / norm_len)
	}

	// Orient normal toward viewpoint: dot(normal, viewpoint - mean) > 0
	view_dir := viewpoint - mean
	if core.vec3_dot(normal, view_dir) < 0.0 {
		normal = -normal
	}

	// Curvature score: lambda_min / (lambda_0 + lambda_1 + lambda_2)
	sum_evals := eigenvalues.x + eigenvalues.y + eigenvalues.z
	curvature: f32 = 0.0
	if sum_evals > 1e-6 {
		curvature = math.clamp(eigenvalues.z / sum_evals, 0.0, 1.0 / 3.0)
	}

	return NormalResult{
		normal    = normal,
		curvature = curvature,
		valid     = true,
	}
}

//
// Estimate and populate surface normals for all points in a PointCloud.
//
normal_estimator_compute :: proc(
	est: ^NormalEstimator,
	cloud: ^sensor.PointCloud,
) {
	n := len(cloud.points)
	if n < 3 do return

	// 1. Build spatial hash table
	for i in 0 ..< len(est.hash_bins) {
		est.hash_bins[i] = -1
	}

	inv_cell := 1.0 / est.cell_size
	entry_count := math.min(n, len(est.entry_pool))

	for i in 0 ..< entry_count {
		p := cloud.points[i].position
		gx := i32(math.floor(p.x * inv_cell))
		gy := i32(math.floor(p.y * inv_cell))
		gz := i32(math.floor(p.z * inv_cell))

		key := voxel_hash_coords(gx, gy, gz)
		bin := int(key) & est.bin_mask

		est.entry_pool[i] = NeighborCell{
			key         = key,
			point_index = i,
			next_in_bin = est.hash_bins[bin],
			occupied    = true,
		}
		est.hash_bins[bin] = i
	}

	radius_sq := est.search_radius * est.search_radius
	k_target := est.k_neighbors

	neighbor_buffer := make([dynamic]core.Vec3, 0, k_target * 2)
	defer delete(neighbor_buffer)

	// 2. Compute normals for each point
	for i in 0 ..< entry_count {
		p := cloud.points[i].position
		gx := i32(math.floor(p.x * inv_cell))
		gy := i32(math.floor(p.y * inv_cell))
		gz := i32(math.floor(p.z * inv_cell))

		clear(&neighbor_buffer)

		// Search 3x3x3 neighboring cells
		for dx in -1 ..= 1 {
			for dy in -1 ..= 1 {
				for dz in -1 ..= 1 {
					n_key := voxel_hash_coords(gx + i32(dx), gy + i32(dy), gz + i32(dz))
					bin := int(n_key) & est.bin_mask

					curr := est.hash_bins[bin]
					for curr != -1 {
						cell := est.entry_pool[curr]
						if cell.key == n_key {
							np := cloud.points[cell.point_index].position
							if core.vec3_distance_sq(p, np) <= radius_sq {
								append(&neighbor_buffer, np)
								if len(neighbor_buffer) >= k_target * 2 do break
							}
						}
						curr = cell.next_in_bin
					}
					if len(neighbor_buffer) >= k_target * 2 do break
				}
				if len(neighbor_buffer) >= k_target * 2 do break
			}
			if len(neighbor_buffer) >= k_target * 2 do break
		}

		if len(neighbor_buffer) >= 3 {
			res := estimate_normal_pca(neighbor_buffer[:], est.viewpoint)
			cloud.points[i].normal = res.normal
		} else {
			// Fallback: vertical normal
			cloud.points[i].normal = core.Vec3{0, 0, 1}
		}
	}
}
