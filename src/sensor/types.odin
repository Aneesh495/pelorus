package pelorus_sensor

import core "../core"
import mem "../core/memory"

//
// Common sensor metadata header accompanying all sensory packets.
//
SensorHeader :: struct {
	sequence_id: u64,
	timestamp:   f64,        // Unix epoch timestamp in seconds
	frame_id:    core.FrameId,
}

//
// Individual 3D LiDAR point representation (Array of Structures form).
//
Point3D :: struct {
	position:  core.Vec3,
	intensity: f32,
	timestamp: f64, // Exact laser firing timestamp relative to scan start
	ring:      u16, // Laser channel/ring index
	normal:    core.Vec3,
}

//
// Full 3D Point Cloud container supporting both AOS and cache-aligned SOA access.
//
PointCloud :: struct {
	header:      SensorHeader,
	points:      [dynamic]Point3D,
	soa:         mem.PointBufferSOA,
	owns_memory: bool,
}

//
// High-frequency 6-axis Inertial Measurement Unit (IMU) packet.
//
ImuMeasurement :: struct {
	header:              SensorHeader,
	angular_velocity:    core.Vec3, // Gyroscope in rad/s (body frame)
	linear_acceleration: core.Vec3, // Accelerometer in m/s^2 (body frame)
}

//
// Complete 15-state navigation state estimate for sensor fusion and state estimation.
// State vector: [p (position), v (velocity), q (orientation), b_a (accel bias), b_g (gyro bias)].
//
NavState :: struct {
	timestamp:   f64,
	pose:        core.Transform3D,
	velocity:    core.Vec3,
	accel_bias:  core.Vec3,
	gyro_bias:   core.Vec3,
	gravity:     core.Vec3, // Nominal gravity vector in world frame
}

//
// Health and tracking status flags for LiDAR-inertial odometry.
//
OdometryStatus :: enum u8 {
	Initializing,
	NominalTracking,
	DegenerateMotion,
	HighUncertainty,
	TrackingLost,
}

//
// High-level odometry output packet published to controllers and mapping backends.
//
OdometryPacket :: struct {
	header:              SensorHeader,
	pose:                core.Transform3D,
	twist:               core.Twist6, // Body-frame linear and angular velocity
	status:              OdometryStatus,
	inlier_ratio:        f32,
	iterations_to_converge: u32,
}

//
// Initialize a PointCloud with pre-allocated capacity.
//
point_cloud_init :: proc(cloud: ^PointCloud, capacity: int) {
	cloud.points = make([dynamic]Point3D, 0, capacity)
	mem.soa_init(&cloud.soa, capacity)
	cloud.owns_memory = true
	cloud.header.sequence_id = 0
	cloud.header.timestamp = 0.0
	cloud.header.frame_id = .SensorLidar
}

//
// Destroy PointCloud and release allocated memory.
//
point_cloud_destroy :: proc(cloud: ^PointCloud) {
	if cloud.owns_memory {
		delete(cloud.points)
		mem.soa_destroy(&cloud.soa)
	}
	cloud.points = nil
	cloud.owns_memory = false
}

//
// Clear all points from cloud without releasing allocated capacity.
//
point_cloud_clear :: proc(cloud: ^PointCloud) {
	clear(&cloud.points)
	mem.soa_clear(&cloud.soa)
}

//
// Append a point to the point cloud, populating both AOS and SOA representations.
//
point_cloud_push :: proc(
	cloud: ^PointCloud,
	pos: core.Vec3,
	intensity: f32 = 0.0,
	timestamp: f64 = 0.0,
	ring: u16 = 0,
	normal: core.Vec3 = core.Vec3{0, 0, 0},
) {
	pt := Point3D{
		position  = pos,
		intensity = intensity,
		timestamp = timestamp,
		ring      = ring,
		normal    = normal,
	}
	append(&cloud.points, pt)
	mem.soa_push(&cloud.soa, pos.x, pos.y, pos.z, intensity, timestamp, ring)
}

//
// Synchronize SOA buffer from AOS points array.
//
point_cloud_sync_soa :: proc(cloud: ^PointCloud) {
	mem.soa_clear(&cloud.soa)
	for pt in cloud.points {
		mem.soa_push(&cloud.soa, pt.position.x, pt.position.y, pt.position.z, pt.intensity, pt.timestamp, pt.ring)
	}
}

//
// Number of valid points in cloud.
//
point_cloud_len :: proc "contextless" (cloud: ^PointCloud) -> int {
	return len(cloud.points)
}
