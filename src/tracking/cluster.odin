package pelorus_tracking

import "core:math"
import core "../core"
import sensor "../sensor"

//
// Individual 3D point cluster extracted from obstacle point cloud.
//
PointCluster :: struct {
	id:       int,
	centroid: core.Vec3,
	points:   [dynamic]core.Vec3,
	bounds:   core.AABB,
}

//
// 3D Euclidean Cluster Extractor using spatial hash indexing and BFS flood-fill.
// Clusters non-ground obstacle points into discrete physical entities (vehicles, pedestrians, cyclists).
//
EuclideanClusterer :: struct {
	cluster_tolerance: f32, // Maximum spatial Euclidean distance to join adjacent points (meters, e.g. 0.5)
	min_cluster_size:  int, // Minimum point count to form a valid obstacle (e.g. 5)
	max_cluster_size:  int, // Maximum point count to avoid merging large walls (e.g. 5000)
	cell_size:         f32,
	hash_bins:         []int,
	bin_mask:          int,
	entry_pool:        []core.SpatialNeighborCell,
	visited:           []bool,
	owns_memory:       bool,
}

//
// Initialize EuclideanClusterer.
//
clusterer_init :: proc(
	clusterer: ^EuclideanClusterer,
	cluster_tolerance: f32 = 0.5,
	min_size: int = 5,
	max_size: int = 5000,
	capacity: int = 16384,
) {
	clusterer.cluster_tolerance = cluster_tolerance > 0.05 ? cluster_tolerance : 0.5
	clusterer.min_cluster_size = min_size
	clusterer.max_cluster_size = max_size
	clusterer.cell_size = clusterer.cluster_tolerance

	table_cap := 1024
	for table_cap < capacity * 2 {
		table_cap *= 2
	}

	clusterer.hash_bins = make([]int, table_cap)
	clusterer.bin_mask = table_cap - 1
	clusterer.entry_pool = make([]core.SpatialNeighborCell, capacity)
	clusterer.visited = make([]bool, capacity)
	clusterer.owns_memory = true
}

//
// Destroy clusterer.
//
clusterer_destroy :: proc(clusterer: ^EuclideanClusterer) {
	if clusterer.owns_memory {
		if clusterer.hash_bins != nil do delete(clusterer.hash_bins)
		if clusterer.entry_pool != nil do delete(clusterer.entry_pool)
		if clusterer.visited != nil do delete(clusterer.visited)
	}
	clusterer.hash_bins = nil
	clusterer.entry_pool = nil
	clusterer.visited = nil
}

//
// Extract discrete Euclidean clusters from input obstacle PointCloud.
// Returns array of segmented point clusters.
//
clusterer_extract :: proc(
	clusterer: ^EuclideanClusterer,
	clusters_out: ^[dynamic]PointCluster,
	cloud: ^sensor.PointCloud,
) -> int {
	n := len(cloud.points)
	if n < clusterer.min_cluster_size do return 0

	entry_count := math.min(n, len(clusterer.entry_pool))

	// 1. Build spatial hash table
	for i in 0 ..< len(clusterer.hash_bins) {
		clusterer.hash_bins[i] = -1
	}

	inv_cell := 1.0 / clusterer.cell_size
	for i in 0 ..< entry_count {
		p := cloud.points[i].position
		gx := i32(math.floor(p.x * inv_cell))
		gy := i32(math.floor(p.y * inv_cell))
		gz := i32(math.floor(p.z * inv_cell))

		key := core.voxel_hash_coords(gx, gy, gz)
		bin := int(key) & clusterer.bin_mask

		clusterer.entry_pool[i] = core.SpatialNeighborCell{
			key         = key,
			point_index = i,
			next_in_bin = clusterer.hash_bins[bin],
			occupied    = true,
		}
		clusterer.hash_bins[bin] = i
		clusterer.visited[i] = false
	}

	bfs_queue := make([dynamic]int, 0, 512)
	defer delete(bfs_queue)

	tolerance_sq := clusterer.cluster_tolerance * clusterer.cluster_tolerance
	cluster_id := 0

	// 2. BFS flood-fill clustering
	for i in 0 ..< entry_count {
		if clusterer.visited[i] do continue

		clear(&bfs_queue)
		append(&bfs_queue, i)
		clusterer.visited[i] = true

		current_pts := make([dynamic]core.Vec3, 0, 64)
		box := core.aabb_empty()
		sum_pos := core.vec3_zero()

		q_head := 0
		for q_head < len(bfs_queue) {
			curr_idx := bfs_queue[q_head]
			q_head += 1

			curr_p := cloud.points[curr_idx].position
			append(&current_pts, curr_p)
			core.aabb_expand_point(&box, curr_p)
			sum_pos += curr_p

			if len(current_pts) >= clusterer.max_cluster_size do break

			gx := i32(math.floor(curr_p.x * inv_cell))
			gy := i32(math.floor(curr_p.y * inv_cell))
			gz := i32(math.floor(curr_p.z * inv_cell))

			// Search 3x3x3 neighboring cells
			for dx in -1 ..= 1 {
				for dy in -1 ..= 1 {
					for dz in -1 ..= 1 {
						n_key := core.voxel_hash_coords(gx + i32(dx), gy + i32(dy), gz + i32(dz))
						bin := int(n_key) & clusterer.bin_mask

						curr_node := clusterer.hash_bins[bin]
						for curr_node != -1 {
							cell := clusterer.entry_pool[curr_node]
							if cell.key == n_key && !clusterer.visited[cell.point_index] {
								np := cloud.points[cell.point_index].position
								if core.vec3_distance_sq(curr_p, np) <= tolerance_sq {
									clusterer.visited[cell.point_index] = true
									append(&bfs_queue, cell.point_index)
								}
							}
							curr_node = cell.next_in_bin
						}
					}
				}
			}
		}

		if len(current_pts) >= clusterer.min_cluster_size && len(current_pts) <= clusterer.max_cluster_size {
			centroid := sum_pos * (1.0 / f32(len(current_pts)))
			cluster := PointCluster{
				id       = cluster_id,
				centroid = centroid,
				points   = current_pts,
				bounds   = box,
			}
			append(clusters_out, cluster)
			cluster_id += 1
		} else {
			delete(current_pts)
		}
	}

	return len(clusters_out)
}
