package pelorus_tracking

import "core:math"
import core "../core"

//
// 3D Oriented Bounding Box detection result.
//
BoundingBox3D :: struct {
	center:   core.Vec3, // Geometric center in world frame (x, y, z)
	size:     core.Vec3, // Extents: (length along heading, width lateral, height vertical)
	yaw:      f32,       // Heading orientation angle around Z-axis (radians)
	obb:      core.OBB,  // Underlying geometric OBB representation
}

//
// Fit tight 3D Oriented Bounding Box around a point cluster using Principal Component Analysis (PCA).
// Aligns primary box axis along the dominant horizontal eigenvector in the XY plane.
//
fit_obb_pca :: proc "contextless" (points: []core.Vec3) -> BoundingBox3D {
	n := len(points)
	if n == 0 {
		return BoundingBox3D{
			center = core.vec3_zero(),
			size   = core.vec3_zero(),
			yaw    = 0.0,
			obb    = core.obb_from_center_size_yaw(core.vec3_zero(), core.vec3_zero(), 0.0),
		}
	}

	// 1. Compute 2D centroid in XY and vertical Z bounds
	mean_x: f32 = 0.0
	mean_y: f32 = 0.0
	min_z: f32 = points[0].z
	max_z: f32 = points[0].z

	for p in points {
		mean_x += p.x
		mean_y += p.y
		if p.z < min_z do min_z = p.z
		if p.z > max_z do max_z = p.z
	}

	inv_n := 1.0 / f32(n)
	mean_x *= inv_n
	mean_y *= inv_n

	if n < 3 {
		center := core.Vec3{mean_x, mean_y, (min_z + max_z) * 0.5}
		size := core.Vec3{0.5, 0.5, math.max(0.2, max_z - min_z)}
		return BoundingBox3D{
			center = center,
			size   = size,
			yaw    = 0.0,
			obb    = core.obb_from_center_size_yaw(center, size, 0.0),
		}
	}

	// 2. Compute 2D horizontal covariance matrix in XY
	cov_xx: f32 = 0.0
	cov_yy: f32 = 0.0
	cov_xy: f32 = 0.0

	for p in points {
		dx := p.x - mean_x
		dy := p.y - mean_y
		cov_xx += dx * dx
		cov_yy += dy * dy
		cov_xy += dx * dy
	}

	scale := 1.0 / f32(n - 1)
	cov_xx *= scale
	cov_yy *= scale
	cov_xy *= scale

	// 3. Eigen-decomposition of 2x2 symmetric matrix
	// Characteristic polynomial: lambda^2 - tr * lambda + det = 0
	tr := cov_xx + cov_yy
	det := cov_xx * cov_yy - cov_xy * cov_xy
	disc := tr * tr * 0.25 - det
	root_disc := math.sqrt(math.max(0.0, disc))

	lambda1 := tr * 0.5 + root_disc // Dominant eigenvalue

	// Primary horizontal heading direction (eigenvector)
	yaw: f32 = 0.0
	if math.abs(cov_xy) > 1e-5 {
		yaw = math.atan2(lambda1 - cov_xx, cov_xy)
	} else if cov_xx >= cov_yy {
		yaw = 0.0
	} else {
		yaw = f32(math.PI * 0.5)
	}

	cos_yaw := math.cos(yaw)
	sin_yaw := math.sin(yaw)

	// Primary axis u = [cos, sin], lateral axis v = [-sin, cos]
	min_u: f32 = 1e30
	max_u: f32 = -1e30
	min_v: f32 = 1e30
	max_v: f32 = -1e30

	for p in points {
		dx := p.x - mean_x
		dy := p.y - mean_y
		u := dx * cos_yaw + dy * sin_yaw
		v := -dx * sin_yaw + dy * cos_yaw

		if u < min_u do min_u = u
		if u > max_u do max_u = u
		if v < min_v do min_v = v
		if v > max_v do max_v = v
	}

	length := math.max(0.1, max_u - min_u)
	width := math.max(0.1, max_v - min_v)
	height := math.max(0.1, max_z - min_z)

	center_u := (min_u + max_u) * 0.5
	center_v := (min_v + max_v) * 0.5

	center_x := mean_x + center_u * cos_yaw - center_v * sin_yaw
	center_y := mean_y + center_u * sin_yaw + center_v * cos_yaw
	center_z := (min_z + max_z) * 0.5

	center := core.Vec3{center_x, center_y, center_z}
	size := core.Vec3{length, width, height}

	return BoundingBox3D{
		center = center,
		size   = size,
		yaw    = yaw,
		obb    = core.obb_from_center_size_yaw(center, size, yaw),
	}
}

//
// Fit oriented bounding boxes across an array of point clusters.
//
fit_bounding_boxes :: proc(clusters: []PointCluster, boxes_out: ^[dynamic]BoundingBox3D) {
	for cluster in clusters {
		box := fit_obb_pca(cluster.points[:])
		append(boxes_out, box)
	}
}
