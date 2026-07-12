package pelorus_tests

import "core:math"
import "core:testing"
import core "../src/core"
import sensor "../src/sensor"
import tracking "../src/tracking"

@(test)
test_euclidean_clustering :: proc(t: ^testing.T) {
	clusterer: tracking.EuclideanClusterer
	tracking.clusterer_init(&clusterer, 0.5, 4, 100)
	defer tracking.clusterer_destroy(&clusterer)

	cloud: sensor.PointCloud
	sensor.point_cloud_init(&cloud, 50)
	defer sensor.point_cloud_destroy(&cloud)

	// Cluster A around (0, 0, 0): 5 points
	sensor.point_cloud_push(&cloud, core.Vec3{0.0, 0.0, 0.0})
	sensor.point_cloud_push(&cloud, core.Vec3{0.1, 0.0, 0.0})
	sensor.point_cloud_push(&cloud, core.Vec3{0.0, 0.1, 0.0})
	sensor.point_cloud_push(&cloud, core.Vec3{-0.1, 0.0, 0.0})
	sensor.point_cloud_push(&cloud, core.Vec3{0.0, -0.1, 0.0})

	// Cluster B around (5.0, 5.0, 0.0): 5 points
	sensor.point_cloud_push(&cloud, core.Vec3{5.0, 5.0, 0.0})
	sensor.point_cloud_push(&cloud, core.Vec3{5.1, 5.0, 0.0})
	sensor.point_cloud_push(&cloud, core.Vec3{5.0, 5.1, 0.0})
	sensor.point_cloud_push(&cloud, core.Vec3{4.9, 5.0, 0.0})
	sensor.point_cloud_push(&cloud, core.Vec3{5.0, 4.9, 0.0})

	clusters := make([dynamic]tracking.PointCluster, 0, 10)
	defer {
		for i in 0 ..< len(clusters) do delete(clusters[i].points)
		delete(clusters)
	}

	count := tracking.clusterer_extract(&clusterer, &clusters, &cloud)
	testing.expect_value(t, count, 2)

	// Check centroids
	c0 := clusters[0].centroid
	c1 := clusters[1].centroid
	dist0 := core.vec3_distance(c0, core.Vec3{0, 0, 0})
	dist1 := core.vec3_distance(c1, core.Vec3{5, 5, 0})
	testing.expect(t, dist0 < 0.1 || core.vec3_distance(c0, core.Vec3{5, 5, 0}) < 0.1, "Cluster centroid 0 mismatch")
	testing.expect(t, dist1 < 0.1 || core.vec3_distance(c1, core.Vec3{0, 0, 0}) < 0.1, "Cluster centroid 1 mismatch")
}

@(test)
test_obb_fitting :: proc(t: ^testing.T) {
	// Points representing a 4.0m x 2.0m vehicle oriented along X axis
	pts := [8]core.Vec3{
		{-2.0, -1.0, 0.0},
		{ 2.0, -1.0, 0.0},
		{ 2.0,  1.0, 0.0},
		{-2.0,  1.0, 0.0},
		{-2.0, -1.0, 1.5},
		{ 2.0, -1.0, 1.5},
		{ 2.0,  1.0, 1.5},
		{-2.0,  1.0, 1.5},
	}

	box := tracking.fit_obb_pca(pts[:])

	testing.expect(t, math.abs(box.center.x) < 0.1, "OBB Center X mismatch")
	testing.expect(t, math.abs(box.center.y) < 0.1, "OBB Center Y mismatch")
	testing.expect(t, math.abs(box.center.z - 0.75) < 0.1, "OBB Center Z mismatch")

	// Dimensions: length ~ 4.0, width ~ 2.0, height ~ 1.5
	testing.expect(t, math.abs(box.size.x - 4.0) < 0.2, "OBB Length mismatch")
	testing.expect(t, math.abs(box.size.y - 2.0) < 0.2, "OBB Width mismatch")
	testing.expect(t, math.abs(box.size.z - 1.5) < 0.1, "OBB Height mismatch")
}

@(test)
test_ctrv_kalman_filter :: proc(t: ^testing.T) {
	kf: tracking.KalmanFilter3D
	tracking.kalman3d_init(&kf, core.Vec3{0, 0, 0}, 0.0, core.Vec3{4, 2, 1.5}, 5.0)

	// Speed = 5.0 m/s heading east along +X, dt = 0.1s
	// Predict 10 steps (1.0 second)
	for _ in 0 ..< 10 {
		tracking.kalman3d_predict(&kf, 0.1)
	}

	// Should have advanced ~5.0m along X
	testing.expect(t, math.abs(kf.state[0] - 5.0) < 0.1, "Linear CTRV prediction distance mismatch")
	testing.expect(t, math.abs(kf.state[1]) < 0.1, "Lateral Y position should remain near zero")

	// Measurement update at (5.2, 0.1, 0.0)
	tracking.kalman3d_update(&kf, core.Vec3{5.2, 0.1, 0.0}, 0.0, core.Vec3{4, 2, 1.5})
	testing.expect(t, kf.state[0] > 5.0, "State X position should be updated by measurement")
}

@(test)
test_hungarian_association :: proc(t: ^testing.T) {
	cost: [tracking.MAX_TRACKS][tracking.MAX_DETECTIONS]f32
	// Track 0 close to Det 0 (cost 0.2), far from Det 1 (cost 5.0)
	// Track 1 close to Det 1 (cost 0.3), far from Det 0 (cost 6.0)
	cost[0][0] = 0.2
	cost[0][1] = 5.0
	cost[1][0] = 6.0
	cost[1][1] = 0.3

	res := tracking.hungarian_solve(&cost, 2, 2, 4.0)
	defer {
		delete(res.track_matches)
		delete(res.unmatched_tracks)
		delete(res.unmatched_detections)
	}

	testing.expect_value(t, len(res.track_matches), 2)
	// Track 0 -> Det 0, Track 1 -> Det 1
	testing.expect_value(t, res.track_matches[0][0], 0)
	testing.expect_value(t, res.track_matches[0][1], 0)
	testing.expect_value(t, res.track_matches[1][0], 1)
	testing.expect_value(t, res.track_matches[1][1], 1)
}

@(test)
test_tracker_lifecycle :: proc(t: ^testing.T) {
	mot: tracking.MultiObjectTracker
	tracking.tracker_init(&mot, 2, 2, 4.0)
	defer tracking.tracker_destroy(&mot)

	// Frame 1: Detection at (1, 0, 0) -> spawns tentative track
	det1 := [1]tracking.BoundingBox3D{
		{center = core.Vec3{1.0, 0.0, 0.0}, size = core.Vec3{4, 2, 1.5}, yaw = 0.0},
	}
	tracking.tracker_step(&mot, det1[:], 0.1)
	testing.expect_value(t, len(mot.tracks), 1)
	testing.expect_value(t, mot.tracks[0].state, tracking.TrackState.Tentative)

	// Frame 2: Detection moves to (1.5, 0, 0) -> confirmed track (hits = 2)
	det2 := [1]tracking.BoundingBox3D{
		{center = core.Vec3{1.5, 0.0, 0.0}, size = core.Vec3{4, 2, 1.5}, yaw = 0.0},
	}
	tracking.tracker_step(&mot, det2[:], 0.1)
	testing.expect_value(t, len(mot.tracks), 1)
	testing.expect_value(t, mot.tracks[0].state, tracking.TrackState.Confirmed)

	// Frame 3: No detections -> track coasts
	empty_dets := [0]tracking.BoundingBox3D{}
	tracking.tracker_step(&mot, empty_dets[:], 0.1)
	testing.expect_value(t, len(mot.tracks), 1)
	testing.expect_value(t, mot.tracks[0].state, tracking.TrackState.Coasting)

	// Frames 4, 5, 6: No detections past max_coasting_frames -> track terminates and is pruned
	tracking.tracker_step(&mot, empty_dets[:], 0.1)
	tracking.tracker_step(&mot, empty_dets[:], 0.1)
	tracking.tracker_step(&mot, empty_dets[:], 0.1)
	testing.expect_value(t, len(mot.tracks), 0)
}
