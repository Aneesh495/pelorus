package pelorus_sensor

import "core:math"
import core "../core"

//
// Standard automotive/industrial LiDAR packet constants (Velodyne / RoboSense layout).
//
LIDAR_PACKET_PAYLOAD_BYTES :: 1206
LIDAR_BLOCKS_PER_PACKET :: 12
LIDAR_CHANNELS_PER_BLOCK :: 32
LIDAR_BLOCK_FLAG :: 0xEEFF // Little-endian 0xFFEE

//
// Factory return mode.
//
ReturnMode :: enum u8 {
	Strongest = 0x37,
	Last      = 0x38,
	Dual      = 0x39,
}

//
// High-throughput packet stream decoder for standard spinning LiDAR UDP packets.
// Precomputes vertical elevation trigonometric tables for zero-trig inner-loop decoding.
//
LidarPacketParser :: struct {
	sin_elevation:    [32]f32,
	cos_elevation:    [32]f32,
	channel_count:    int,
	distance_scale:   f32, // Meter conversion factor (typically 0.002m per count)
	min_range_m:      f32,
	max_range_m:      f32,
	packets_parsed:   u64,
	points_extracted: u64,
}

//
// Initialize parser with 16-beam vertical profile and precomputed trigonometric tables.
//
lidar_parser_init_vlp16 :: proc(parser: ^LidarPacketParser) {
	parser.channel_count = 16
	parser.distance_scale = 0.002 // 2mm resolution
	parser.min_range_m = 0.4
	parser.max_range_m = 120.0
	parser.packets_parsed = 0
	parser.points_extracted = 0

	elevations := [16]f32{
		-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0,
		-7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0,
	}

	deg_to_rad := f32(math.PI / 180.0)
	for i in 0 ..< 16 {
		rad := elevations[i] * deg_to_rad
		parser.sin_elevation[i] = math.sin(rad)
		parser.cos_elevation[i] = math.cos(rad)
		// Duplicate for upper bank of 32-channel block
		parser.sin_elevation[i + 16] = parser.sin_elevation[i]
		parser.cos_elevation[i + 16] = parser.cos_elevation[i]
	}
}

//
// Parse a raw 1206-byte UDP packet and append extracted valid Cartesian points to PointCloud.
//
lidar_parser_parse_packet :: proc(
	parser: ^LidarPacketParser,
	packet_data: []u8,
	cloud: ^PointCloud,
	packet_timestamp: f64,
) -> int {
	if len(packet_data) < LIDAR_PACKET_PAYLOAD_BYTES {
		return 0
	}

	points_added := 0
	offset := 0
	deg_to_rad := f32(math.PI / 180.0)

	for block := 0; block < LIDAR_BLOCKS_PER_PACKET; block += 1 {
		// Read 2-byte header flag
		flag := u16(packet_data[offset]) | (u16(packet_data[offset + 1]) << 8)
		if flag != LIDAR_BLOCK_FLAG {
			offset += 100
			continue
		}

		// Read 2-byte azimuth in hundredths of a degree
		raw_azimuth := u16(packet_data[offset + 2]) | (u16(packet_data[offset + 3]) << 8)
		azimuth_rad := f32(raw_azimuth) * 0.01 * deg_to_rad
		sin_azimuth := math.sin(azimuth_rad)
		cos_azimuth := math.cos(azimuth_rad)

		channel_offset := offset + 4
		for ch := 0; ch < parser.channel_count; ch += 1 {
			// Read 2-byte distance
			dist_raw := u16(packet_data[channel_offset]) | (u16(packet_data[channel_offset + 1]) << 8)
			intensity_raw := f32(packet_data[channel_offset + 2])
			channel_offset += 3

			distance := f32(dist_raw) * parser.distance_scale
			if distance < parser.min_range_m || distance > parser.max_range_m {
				continue
			}

			// Project spherical to Cartesian:
			// x = r * cos(elev) * sin(azimuth)
			// y = r * cos(elev) * cos(azimuth)
			// z = r * sin(elev)
			cos_elev := parser.cos_elevation[ch]
			sin_elev := parser.sin_elevation[ch]

			xy_dist := distance * cos_elev
			x := xy_dist * sin_azimuth
			y := xy_dist * cos_azimuth
			z := distance * sin_elev

			// Time offset within 1.2ms packet
			time_offset := f64(block) * 1e-4 + f64(ch) * 3e-6
			pt_time := packet_timestamp + time_offset

			point_cloud_push(cloud, core.Vec3{x, y, z}, intensity_raw, pt_time, u16(ch))
			points_added += 1
		}

		offset += 100
	}

	parser.packets_parsed += 1
	parser.points_extracted += u64(points_added)
	return points_added
}

//
// Synthetic packet generator: formats synthetic points into standard 1206-byte UDP packets
// for hardware-in-the-loop simulation and zero-dependency offline replay.
//
lidar_generate_synthetic_packet :: proc(
	packet_out: []u8,
	azimuth_start_deg: f32,
	distance_m: f32,
) {
	if len(packet_out) < LIDAR_PACKET_PAYLOAD_BYTES do return

	offset := 0
	dist_units := u16(distance_m / 0.002)

	for b := 0; b < LIDAR_BLOCKS_PER_PACKET; b += 1 {
		// Header flag 0xFFEE
		packet_out[offset] = 0xFF
		packet_out[offset + 1] = 0xEE

		// Azimuth increment 0.2 degrees per block
		azimuth := azimuth_start_deg + f32(b) * 0.2
		for azimuth >= 360.0 do azimuth -= 360.0
		azimuth_val := u16(azimuth * 100.0)

		packet_out[offset + 2] = u8(azimuth_val & 0xFF)
		packet_out[offset + 3] = u8((azimuth_val >> 8) & 0xFF)

		channel_offset := offset + 4
		for ch := 0; ch < 32; ch += 1 {
			packet_out[channel_offset] = u8(dist_units & 0xFF)
			packet_out[channel_offset + 1] = u8((dist_units >> 8) & 0xFF)
			packet_out[channel_offset + 2] = 128 // Default intensity
			channel_offset += 3
		}

		offset += 100
	}

	// 4-byte timestamp
	packet_out[1200] = 0
	packet_out[1201] = 0
	packet_out[1202] = 0
	packet_out[1203] = 0

	// Factory bytes
	packet_out[1204] = u8(ReturnMode.Strongest)
	packet_out[1205] = 0x22 // VLP-16 model code
}
