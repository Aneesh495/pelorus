package pelorus_tests

import "core:math"
import "core:testing"
import core "../src/core"
import filter "../src/filter"
import sensor "../src/sensor"

@(test)
test_voxel_grid_filter :: proc(t: ^testing.T) {
	vg: filter.VoxelGridFilter
	filter.voxel_grid_init(&vg, core.Vec3{0.5, 0.5, 0.5}, 100)
	defer filter.voxel_grid_destroy(&vg)

	cloud_in: sensor.PointCloud
	sensor.point_cloud_init(&cloud_in, 10)
	defer sensor.point_cloud_destroy(&cloud_in)

	// Add 4 points all inside the first voxel [0, 0.5]^3
	sensor.point_cloud_push(&cloud_in, core.Vec3{0.1, 0.1, 0.1}, 10.0)
	sensor.point_cloud_push(&cloud_in, core.Vec3{0.2, 0.1, 0.1}, 20.0)
	sensor.point_cloud_push(&cloud_in, core.Vec3{0.1, 0.2, 0.1}, 30.0)
	sensor.point_cloud_push(&cloud_in, core.Vec3{0.2, 0.2, 0.1}, 40.0)

	// Add 1 point in a different voxel [1.0, 1.5]^3
	sensor.point_cloud_push(&cloud_in, core.Vec3{1.2, 1.2, 1.2}, 50.0)

	cloud_out: sensor.PointCloud
	sensor.point_cloud_init(&cloud_out, 10)
	defer sensor.point_cloud_destroy(&cloud_out)

	out_count := filter.voxel_grid_filter(&vg, &cloud_out, &cloud_in)
	testing.expect_value(t, out_count, 2)
	testing.expect_value(t, sensor.point_cloud_len(&cloud_out), 2)

	// Check centroid of the 4 points: x = (0.1+0.2+0.1+0.2)/4 = 0.15, y = 0.15, z = 0.1
	// Find the downsampled point near (0.15, 0.15, 0.1)
	found_centroid := false
	for pt in cloud_out.points {
		if core.vec3_distance(pt.position, core.Vec3{0.15, 0.15, 0.1}) < 1e-4 {
			found_centroid = true
			testing.expect(t, math.abs(pt.intensity - 25.0) < 1e-4, "Centroid mean intensity mismatch")
		}
	}
	testing.expect(t, found_centroid, "Downsampled voxel centroid mismatch")
}

@(test)
test_passthrough_filter :: proc(t: ^testing.T) {
	pt_filter := filter.passthrough_default()
	pt_filter.min_bounds = core.Vec3{-2.0, -2.0, -1.0}
	pt_filter.max_bounds = core.Vec3{2.0, 2.0, 1.0}
	pt_filter.min_radius = 0.5
	pt_filter.max_radius = 5.0

	cloud_in: sensor.PointCloud
	sensor.point_cloud_init(&cloud_in, 10)
	defer sensor.point_cloud_destroy(&cloud_in)

	// Point 0: inside all bounds (x=1.0, r=1.0) -> keep
	sensor.point_cloud_push(&cloud_in, core.Vec3{1.0, 0.0, 0.0})
	// Point 1: too close to origin (r=0.2 < min_radius) -> discard
	sensor.point_cloud_push(&cloud_in, core.Vec3{0.1, 0.1, 0.0})
	// Point 2: outside AABB bounds (x=10.0 > max_bounds.x) -> discard
	sensor.point_cloud_push(&cloud_in, core.Vec3{10.0, 0.0, 0.0})

	cloud_out: sensor.PointCloud
	sensor.point_cloud_init(&cloud_out, 10)
	defer sensor.point_cloud_destroy(&cloud_out)

	retained := filter.filter_passthrough(&pt_filter, &cloud_out, &cloud_in)
	testing.expect_value(t, retained, 1)
	testing.expect_value(t, sensor.point_cloud_len(&cloud_out), 1)
	testing.expect_value(t, cloud_out.points[0].position.x, 1.0)
}

@(test)
test_ground_plane_segmentation :: proc(t: ^testing.T) {
	seg: filter.GroundSegmenter
	filter.ground_segmenter_init(&seg, 0.15, 15.0, -0.1)

	cloud_in: sensor.PointCloud
	sensor.point_cloud_init(&cloud_in, 100)
	defer sensor.point_cloud_destroy(&cloud_in)

	// Generate 30 ground points on plane z = -0.5
	for x in -2 ..= 2 {
		for y in -2 ..= 3 {
			sensor.point_cloud_push(&cloud_in, core.Vec3{f32(x) * 0.5, f32(y) * 0.5, -0.5})
		}
	}

	// Generate 5 obstacle points elevated at z = 1.5
	sensor.point_cloud_push(&cloud_in, core.Vec3{0.0, 1.0, 1.5})
	sensor.point_cloud_push(&cloud_in, core.Vec3{0.2, 1.0, 1.5})
	sensor.point_cloud_push(&cloud_in, core.Vec3{-0.2, 1.0, 1.5})

	ground_cloud: sensor.PointCloud
	sensor.point_cloud_init(&ground_cloud, 100)
	defer sensor.point_cloud_destroy(&ground_cloud)

	obstacle_cloud: sensor.PointCloud
	sensor.point_cloud_init(&obstacle_cloud, 100)
	defer sensor.point_cloud_destroy(&obstacle_cloud)

	res := filter.ground_segmenter_segment(&seg, &ground_cloud, &obstacle_cloud, &cloud_in)

	testing.expect(t, res.inlier_count >= 25, "Ground inlier count should capture planar points")
	testing.expect(t, res.obstacle_points >= 3, "Obstacle points should capture elevated points")
	testing.expect(t, math.abs(res.ground_plane.normal.z - 1.0) < 0.1, "Ground normal should point along +Z")
}

@(test)
test_normal_estimator :: proc(t: ^testing.T) {
	est: filter.NormalEstimator
	filter.normal_estimator_init(&est, 10, 1.0, core.Vec3{0, 0, 5}, 100)
	defer filter.normal_estimator_destroy(&est)

	cloud: sensor.PointCloud
	sensor.point_cloud_init(&cloud, 50)
	defer sensor.point_cloud_destroy(&cloud)

	// Points lying exactly on horizontal plane z = 0
	for x in -2 ..= 2 {
		for y in -2 ..= 2 {
			sensor.point_cloud_push(&cloud, core.Vec3{f32(x) * 0.2, f32(y) * 0.2, 0.0})
		}
	}

	filter.normal_estimator_compute(&est, &cloud)

	// Center point at (0, 0, 0) should have normal vector pointing up towards viewpoint (0, 0, 5)
	center_idx := 12 // (0, 0, 0)
	norm := cloud.points[center_idx].normal
	testing.expect(t, math.abs(norm.z - 1.0) < 1e-3, "Surface normal should point along +Z")
	testing.expect(t, math.abs(norm.x) < 1e-3, "Surface normal X component must be near zero")
	testing.expect(t, math.abs(norm.y) < 1e-3, "Surface normal Y component must be near zero")
}
