package pelorus_tests

import "core:math"
import "core:testing"
import core "../src/core"
import sensor "../src/sensor"

@(test)
test_point_cloud_container :: proc(t: ^testing.T) {
	cloud: sensor.PointCloud
	sensor.point_cloud_init(&cloud, 100)
	defer sensor.point_cloud_destroy(&cloud)

	testing.expect_value(t, sensor.point_cloud_len(&cloud), 0)

	sensor.point_cloud_push(&cloud, core.Vec3{1, 2, 3}, 100.0, 1.0, 0)
	sensor.point_cloud_push(&cloud, core.Vec3{4, 5, 6}, 150.0, 1.1, 1)

	testing.expect_value(t, sensor.point_cloud_len(&cloud), 2)
	testing.expect_value(t, cloud.points[0].position.x, 1.0)
	testing.expect_value(t, cloud.points[1].position.z, 6.0)

	// Check that SOA buffer is synchronized
	testing.expect_value(t, cloud.soa.count, 2)
	x, y, z := cloud.soa.xs[1], cloud.soa.ys[1], cloud.soa.zs[1]
	testing.expect_value(t, x, 4.0)
	testing.expect_value(t, y, 5.0)
	testing.expect_value(t, z, 6.0)

	sensor.point_cloud_clear(&cloud)
	testing.expect_value(t, sensor.point_cloud_len(&cloud), 0)
	testing.expect_value(t, cloud.soa.count, 0)
}

@(test)
test_imu_preintegration :: proc(t: ^testing.T) {
	preint: sensor.ImuPreintegrated
	sensor.imu_preintegrated_init(&preint)

	// Pure angular velocity of 1.0 rad/s around Z-axis for 1.0 second (100 steps of 0.01s)
	dt := f32(0.01)
	gyro := core.Vec3{0.0, 0.0, 1.0}
	accel := core.Vec3{0.0, 0.0, 0.0}

	for _ in 0 ..< 100 {
		sensor.imu_preintegrated_integrate(&preint, accel, gyro, dt)
	}

	testing.expect_value(t, preint.sample_count, 100)
	testing.expect(t, math.abs(preint.delta_t - 1.0) < 1e-4, "Accumulated delta_t mismatch")

	// Total rotation angle should be ~1.0 radian around Z
	euler_r, euler_p, euler_y := core.quat_to_euler_xyz(preint.delta_r)
	testing.expect(t, math.abs(euler_y - 1.0) < 1e-2, "Accumulated yaw mismatch")
	testing.expect(t, math.abs(euler_r) < 1e-4, "Roll must be zero")
	testing.expect(t, math.abs(euler_p) < 1e-4, "Pitch must be zero")
}

@(test)
test_point_cloud_deskewing :: proc(t: ^testing.T) {
	// Construct trajectory moving along X axis at 10 m/s
	traj: core.TrajectoryBuffer
	core.trajectory_buffer_init(&traj, 10)
	defer core.trajectory_buffer_destroy(&traj)

	core.trajectory_buffer_push(&traj, core.TrajectoryPose{
		timestamp = 100.0,
		pose      = core.Transform3D{translation = core.Vec3{0, 0, 0}, rotation = core.quat_identity()},
		twist     = core.Twist6{rho = core.Vec3{10, 0, 0}, phi = core.vec3_zero()},
	})
	core.trajectory_buffer_push(&traj, core.TrajectoryPose{
		timestamp = 100.1,
		pose      = core.Transform3D{translation = core.Vec3{1.0, 0, 0}, rotation = core.quat_identity()},
		twist     = core.Twist6{rho = core.Vec3{10, 0, 0}, phi = core.vec3_zero()},
	})

	cloud_in: sensor.PointCloud
	sensor.point_cloud_init(&cloud_in, 10)
	defer sensor.point_cloud_destroy(&cloud_in)

	// Point fired at timestamp 100.1 at local sensor position (5, 0, 0)
	sensor.point_cloud_push(&cloud_in, core.Vec3{5.0, 0, 0}, 100.0, 100.1, 0)

	cloud_out: sensor.PointCloud
	sensor.point_cloud_init(&cloud_out, 10)
	defer sensor.point_cloud_destroy(&cloud_out)

	// Deskew to scan start reference timestamp 100.0
	stats := sensor.deskew_point_cloud(&cloud_out, &cloud_in, &traj, 100.0)

	testing.expect_value(t, stats.points_deskewed, 1)
	testing.expect(t, stats.max_displacement > 0.5, "Deskew must correct motion displacement")

	// Since vehicle moved forward 1.0m between 100.0 and 100.1, point fired at 100.1
	// expressed in 100.0 reference frame should be at x = 5.0 + 1.0 = 6.0m
	deskewed_x := cloud_out.points[0].position.x
	testing.expect(t, math.abs(deskewed_x - 6.0) < 1e-3, "Deskewed X coordinate mismatch")
}

@(test)
test_lidar_packet_parsing :: proc(t: ^testing.T) {
	parser: sensor.LidarPacketParser
	sensor.lidar_parser_init_vlp16(&parser)

	packet_bytes: [sensor.LIDAR_PACKET_PAYLOAD_BYTES]u8
	sensor.lidar_generate_synthetic_packet(packet_bytes[:], 0.0, 10.0)

	cloud: sensor.PointCloud
	sensor.point_cloud_init(&cloud, 1000)
	defer sensor.point_cloud_destroy(&cloud)

	points_parsed := sensor.lidar_parser_parse_packet(&parser, packet_bytes[:], &cloud, 1000.0)
	testing.expect(t, points_parsed > 0, "Packet parser should extract points")
	testing.expect_value(t, sensor.point_cloud_len(&cloud), points_parsed)

	// Points at 10m distance should have 3D distance ~10m
	first_pt := cloud.points[0].position
	dist := core.vec3_length(first_pt)
	testing.expect(t, math.abs(dist - 10.0) < 0.1, "Extracted distance mismatch")
}
