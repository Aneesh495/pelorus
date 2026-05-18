package pelorus_filter

import "core:math"
import core "../core"
import sensor "../sensor"

//
// Result of ground plane segmentation.
//
GroundSegmentationResult :: struct {
	ground_plane: core.Plane3D,
	inlier_count: int,
	ground_points: int,
	obstacle_points: int,
}

//
// Progressive Ground Plane Segmentation via Random Sample Consensus (RANSAC).
// Extracts terrain surface and separates obstacles from drivable ground.
//
GroundSegmenter :: struct {
	max_iterations:        int, // Number of RANSAC hypotheses evaluated (e.g. 50)
	distance_threshold:    f32, // Maximum orthogonal distance to plane for inlier consensus (meters)
	max_normal_angle_rad:  f32, // Maximum angular deviation between surface normal and vertical axis
	initial_elevation_max: f32, // Height threshold to seed ground seed points
}

//
// Initialize ground segmenter with operational tolerances.
//
ground_segmenter_init :: proc(
	seg: ^GroundSegmenter,
	distance_threshold: f32 = 0.15,
	max_angle_deg: f32 = 15.0,
	initial_elevation_max: f32 = -0.2,
) {
	seg.max_iterations = 60
	seg.distance_threshold = distance_threshold
	seg.max_normal_angle_rad = max_angle_deg * f32(math.PI / 180.0)
	seg.initial_elevation_max = initial_elevation_max
}

//
// Perform RANSAC ground plane segmentation.
// Separates cloud_in into ground_out (terrain) and obstacle_out (non-ground).
//
ground_segmenter_segment :: proc(
	seg: ^GroundSegmenter,
	ground_out: ^sensor.PointCloud,
	obstacle_out: ^sensor.PointCloud,
	cloud_in: ^sensor.PointCloud,
) -> GroundSegmentationResult {
	res := GroundSegmentationResult{}
	n := len(cloud_in.points)
	if n < 3 do return res

	// 1. Identify low-elevation candidate seed points
	seed_indices := make([dynamic]int, 0, n / 2)
	defer delete(seed_indices)

	for i in 0 ..< n {
		if cloud_in.points[i].position.z <= seg.initial_elevation_max {
			append(&seed_indices, i)
		}
	}

	// Fallback to all points if low-elevation seeds are sparse
	if len(seed_indices) < 10 {
		clear(&seed_indices)
		for i in 0 ..< n {
			append(&seed_indices, i)
		}
	}

	seed_count := len(seed_indices)
	best_inliers := 0
	best_plane := core.Plane3D{normal = core.Vec3{0, 0, 1}, distance = 0}

	up_vector := core.Vec3{0, 0, 1}
	cos_max_angle := math.cos(seg.max_normal_angle_rad)

	// Simple deterministic pseudo-random sequence for reproducible RANSAC
	rng_state := u64(1337)
	next_rng :: proc(state: ^u64) -> u64 {
		state^ = state^ * 6364136223846793005 + 1442695040888963407
		return state^
	}

	// 2. RANSAC hypothesis loop
	for _ in 0 ..< seg.max_iterations {
		if seed_count < 3 do break

		idx0 := seed_indices[int(next_rng(&rng_state) % u64(seed_count))]
		idx1 := seed_indices[int(next_rng(&rng_state) % u64(seed_count))]
		idx2 := seed_indices[int(next_rng(&rng_state) % u64(seed_count))]

		if idx0 == idx1 || idx1 == idx2 || idx0 == idx2 do continue

		p0 := cloud_in.points[idx0].position
		p1 := cloud_in.points[idx1].position
		p2 := cloud_in.points[idx2].position

		v01 := p1 - p0
		v02 := p2 - p0
		normal := core.vec3_cross(v01, v02)
		normal_len := core.vec3_length(normal)
		if normal_len < 1e-4 do continue // Collinear points

		normal = normal * (1.0 / normal_len)

		// Ensure normal points upward toward +Z
		if normal.z < 0.0 {
			normal = -normal
		}

		// Verify normal is roughly vertical
		cos_angle := core.vec3_dot(normal, up_vector)
		if cos_angle < cos_max_angle do continue

		dist := -core.vec3_dot(normal, p0)
		cand_plane := core.Plane3D{normal = normal, distance = dist}

		// Count inliers
		inlier_count := 0
		for pt in cloud_in.points {
			d := math.abs(core.plane3d_signed_distance(cand_plane, pt.position))
			if d <= seg.distance_threshold {
				inlier_count += 1
			}
		}

		if inlier_count > best_inliers {
			best_inliers = inlier_count
			best_plane = cand_plane
		}
	}

	res.ground_plane = best_plane
	res.inlier_count = best_inliers

	// 3. Partition points into ground and obstacle clouds
	ground_count := 0
	obstacle_count := 0

	for pt in cloud_in.points {
		d := core.plane3d_signed_distance(best_plane, pt.position)
		if math.abs(d) <= seg.distance_threshold {
			// Point lies on terrain
			sensor.point_cloud_push(ground_out, pt.position, pt.intensity, pt.timestamp, pt.ring, best_plane.normal)
			ground_count += 1
		} else if d > seg.distance_threshold {
			// Point lies above terrain (obstacle / structure)
			sensor.point_cloud_push(obstacle_out, pt.position, pt.intensity, pt.timestamp, pt.ring, pt.normal)
			obstacle_count += 1
		}
	}

	res.ground_points = ground_count
	res.obstacle_points = obstacle_count
	return res
}
