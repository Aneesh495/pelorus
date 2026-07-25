package pelorus_io

import "core:mem"
import core "../core"
import sensor "../sensor"

//
// Protocol Magic bytes: 'P', 'E', 'L', 'O'
//
WIRE_MAGIC :: 0x50454C4F

//
// Telemetry Frame Header transmitted over network wire.
//
WireFrameHeader :: struct #packed {
	magic:            u32, // Always WIRE_MAGIC
	version:          u16, // Protocol version (1)
	flags:            u16, // Bitflags (bit 0: compressed, bit 1: keyframe)
	sequence_id:      u64, // Monotonically increasing scan sequence
	timestamp_sec:    f64, // Scan timestamp in seconds
	point_count:      u32, // Number of LiDAR points in payload
	obstacle_count:   u32, // Number of tracked obstacle bounding boxes
	trajectory_count: u32, // Number of planned trajectory waypoints
	checksum:         u32, // FNV-1a 32-bit payload hash
}

//
// Wire representation of state estimation pose.
//
WireStatePose :: struct #packed {
	position:    core.Vec3,
	quaternion:  core.Quat,
	velocity:    core.Vec3,
	gyro_bias:   core.Vec3,
	accel_bias:  core.Vec3,
}

//
// Wire point representation (16 bytes: x, y, z, intensity).
//
WirePoint :: struct #packed {
	x:         f32,
	y:         f32,
	z:         f32,
	intensity: f32,
}

//
// Wire tracked dynamic obstacle representation.
//
WireObstacle :: struct #packed {
	id:             u32,
	center:         core.Vec3,
	extents:        core.Vec3, // Length, width, height
	yaw:            f32,
	velocity:       core.Vec3,
	classification: u8,
	_pad:           [3]u8,
}

//
// Wire planned trajectory knot.
//
WireTrajectoryKnot :: struct #packed {
	position:    core.Vec3,
	velocity:    core.Vec3,
	yaw:         f32,
	time_offset: f32,
}

//
// Compute 32-bit FNV-1a hash over byte buffer.
//
compute_fnv1a_32 :: proc "contextless" (data: []u8) -> u32 {
	h: u32 = 0x811C9DC5
	for b in data {
		h ~= u32(b)
		h *= 0x01000193
	}
	return h
}

//
// Serialize full telemetry state into contiguous network byte buffer.
//
serialize_telemetry_frame :: proc(
	seq_id: u64,
	timestamp: f64,
	pose: WireStatePose,
	points: []sensor.Point3D,
	obstacles: []WireObstacle,
	trajectory: []WireTrajectoryKnot,
) -> []u8 {
	hdr_size := size_of(WireFrameHeader)
	pose_size := size_of(WireStatePose)
	pt_size := size_of(WirePoint)
	obs_size := size_of(WireObstacle)
	traj_size := size_of(WireTrajectoryKnot)

	n_pts := u32(len(points))
	n_obs := u32(len(obstacles))
	n_traj := u32(len(trajectory))

	payload_size := pose_size + int(n_pts) * pt_size + int(n_obs) * obs_size + int(n_traj) * traj_size
	total_size := hdr_size + payload_size

	buffer := make([]u8, total_size)

	// Write payload starting at offset hdr_size
	offset := hdr_size

	// 1. Write Pose
	pose_bytes := transmute([size_of(WireStatePose)]u8)pose
	copy(buffer[offset : offset + pose_size], pose_bytes[:])
	offset += pose_size

	// 2. Write Points
	for pt in points {
		wp := WirePoint{
			x         = pt.position.x,
			y         = pt.position.y,
			z         = pt.position.z,
			intensity = pt.intensity,
		}
		dest := buffer[offset : offset + pt_size]
		src := transmute([size_of(WirePoint)]u8)wp
		copy(dest, src[:])
		offset += pt_size
	}

	// 3. Write Obstacles
	for obs in obstacles {
		dest := buffer[offset : offset + obs_size]
		src := transmute([size_of(WireObstacle)]u8)obs
		copy(dest, src[:])
		offset += obs_size
	}

	// 4. Write Trajectory
	for knot in trajectory {
		dest := buffer[offset : offset + traj_size]
		src := transmute([size_of(WireTrajectoryKnot)]u8)knot
		copy(dest, src[:])
		offset += traj_size
	}

	// Compute checksum over payload
	checksum := compute_fnv1a_32(buffer[hdr_size:])

	// Construct and write header
	hdr := WireFrameHeader{
		magic            = WIRE_MAGIC,
		version          = 1,
		flags            = 0,
		sequence_id      = seq_id,
		timestamp_sec    = timestamp,
		point_count      = n_pts,
		obstacle_count   = n_obs,
		trajectory_count = n_traj,
		checksum         = checksum,
	}

	hdr_bytes := transmute([size_of(WireFrameHeader)]u8)hdr
	copy(buffer[:hdr_size], hdr_bytes[:])

	return buffer
}

//
// Validate and parse incoming telemetry frame header.
//
deserialize_telemetry_header :: proc "contextless" (
	buffer: []u8,
) -> (hdr: WireFrameHeader, ok: bool) {
	hdr_size := size_of(WireFrameHeader)
	if len(buffer) < hdr_size do return WireFrameHeader{}, false

	h_ptr := cast(^WireFrameHeader)&buffer[0]
	hdr = h_ptr^

	if hdr.magic != WIRE_MAGIC do return hdr, false
	if hdr.version != 1 do return hdr, false

	expected_payload := size_of(WireStatePose) +
		int(hdr.point_count) * size_of(WirePoint) +
		int(hdr.obstacle_count) * size_of(WireObstacle) +
		int(hdr.trajectory_count) * size_of(WireTrajectoryKnot)

	if len(buffer) < hdr_size + expected_payload do return hdr, false

	// Validate checksum
	actual_checksum := compute_fnv1a_32(buffer[hdr_size : hdr_size + expected_payload])
	if actual_checksum != hdr.checksum do return hdr, false

	return hdr, true
}

//
// Extract pose state from validated telemetry frame buffer.
//
deserialize_telemetry_pose :: proc "contextless" (
	buffer: []u8,
) -> (pose: WireStatePose, ok: bool) {
	hdr_size := size_of(WireFrameHeader)
	pose_size := size_of(WireStatePose)
	if len(buffer) < hdr_size + pose_size do return WireStatePose{}, false

	p_ptr := cast(^WireStatePose)&buffer[hdr_size]
	return p_ptr^, true
}
