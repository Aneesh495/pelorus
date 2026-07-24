package pelorus_io

import "core:fmt"
import "core:math"
import "core:os"
import "core:strconv"
import "core:strings"
import core "../core"
import sensor "../sensor"

//
// PCD file storage format.
//
PcdFormat :: enum {
	ASCII,
	Binary,
}

//
// PCD Header metadata.
//
PcdHeader :: struct {
	version:   f32,
	width:     int,
	height:    int,
	points:    int,
	is_binary: bool,
}

//
// Write PointCloud to PCD file in ASCII format.
//
pcd_write_ascii :: proc(cloud: ^sensor.PointCloud, filepath: string) -> bool {
	b := strings.builder_make()
	defer strings.builder_destroy(&b)

	n_pts := len(cloud.points)

	// Write standard PCD header
	strings.write_string(&b, "# .PCD v0.7 - Point Cloud Data file format\n")
	strings.write_string(&b, "VERSION 0.7\n")
	strings.write_string(&b, "FIELDS x y z intensity timestamp ring\n")
	strings.write_string(&b, "SIZE 4 4 4 4 8 2\n")
	strings.write_string(&b, "TYPE F F F F F U\n")
	strings.write_string(&b, "COUNT 1 1 1 1 1 1\n")
	fmt.sbprintf(&b, "WIDTH %d\n", n_pts)
	strings.write_string(&b, "HEIGHT 1\n")
	strings.write_string(&b, "VIEWPOINT 0 0 0 1 0 0 0\n")
	fmt.sbprintf(&b, "POINTS %d\n", n_pts)
	strings.write_string(&b, "DATA ascii\n")

	// Write point data rows
	for pt in cloud.points {
		fmt.sbprintf(
			&b,
			"%.4f %.4f %.4f %.2f %.6f %d\n",
			pt.position.x,
			pt.position.y,
			pt.position.z,
			pt.intensity,
			pt.timestamp,
			pt.ring,
		)
	}

	out_bytes := transmute([]u8)strings.to_string(b)
	err := os.write_entire_file(filepath, out_bytes)
	return err == nil
}

//
// Binary point layout matching PCD fields: x, y, z, intensity (f32*4), timestamp (f64), ring (u16).
// Packed without padding to match standard LiDAR PCD layout.
//
PcdBinaryPoint :: struct #packed {
	x:         f32,
	y:         f32,
	z:         f32,
	intensity: f32,
	timestamp: f64,
	ring:      u16,
}

//
// Write PointCloud to PCD file in raw binary format.
//
pcd_write_binary :: proc(cloud: ^sensor.PointCloud, filepath: string) -> bool {
	b := strings.builder_make()
	defer strings.builder_destroy(&b)

	n_pts := len(cloud.points)

	strings.write_string(&b, "# .PCD v0.7 - Point Cloud Data file format\n")
	strings.write_string(&b, "VERSION 0.7\n")
	strings.write_string(&b, "FIELDS x y z intensity timestamp ring\n")
	strings.write_string(&b, "SIZE 4 4 4 4 8 2\n")
	strings.write_string(&b, "TYPE F F F F F U\n")
	strings.write_string(&b, "COUNT 1 1 1 1 1 1\n")
	fmt.sbprintf(&b, "WIDTH %d\n", n_pts)
	strings.write_string(&b, "HEIGHT 1\n")
	strings.write_string(&b, "VIEWPOINT 0 0 0 1 0 0 0\n")
	fmt.sbprintf(&b, "POINTS %d\n", n_pts)
	strings.write_string(&b, "DATA binary\n")

	header_str := strings.to_string(b)
	header_bytes := transmute([]u8)header_str

	point_size := size_of(PcdBinaryPoint)
	total_size := len(header_bytes) + n_pts * point_size

	data := make([]u8, total_size)
	defer delete(data)

	// Copy header
	copy(data[:len(header_bytes)], header_bytes)

	// Copy binary points
	offset := len(header_bytes)
	for pt in cloud.points {
		bin_pt := PcdBinaryPoint{
			x         = pt.position.x,
			y         = pt.position.y,
			z         = pt.position.z,
			intensity = pt.intensity,
			timestamp = pt.timestamp,
			ring      = pt.ring,
		}
		dest := data[offset : offset + point_size]
		src := transmute([size_of(PcdBinaryPoint)]u8)bin_pt
		copy(dest, src[:])
		offset += point_size
	}

	err := os.write_entire_file(filepath, data)
	return err == nil
}

//
// Parse PCD file from disk into PointCloud. Supports both ASCII and binary formats.
//
pcd_read_file :: proc(filepath: string, cloud_out: ^sensor.PointCloud) -> bool {
	data, err := os.read_entire_file_from_path(filepath, context.allocator)
	if err != nil do return false
	defer delete(data)

	return pcd_parse_bytes(data, cloud_out)
}

//
// Parse in-memory byte buffer containing PCD file.
//
pcd_parse_bytes :: proc(data: []u8, cloud_out: ^sensor.PointCloud) -> bool {
	// Locate DATA tag to separate header from payload
	text := string(data)
	data_idx := strings.index(text, "DATA ")
	if data_idx == -1 do return false

	line_end := strings.index(text[data_idx:], "\n")
	if line_end == -1 do return false

	data_line := text[data_idx : data_idx + line_end]
	data_line = strings.trim_space(data_line)

	is_binary := strings.contains(data_line, "binary")
	is_ascii := strings.contains(data_line, "ascii")
	if !is_binary && !is_ascii do return false

	// Extract POINTS count from header
	points_idx := strings.index(text[:data_idx], "POINTS ")
	total_points := 0
	if points_idx != -1 {
		p_line_end := strings.index(text[points_idx:], "\n")
		if p_line_end != -1 {
			p_line := text[points_idx + 7 : points_idx + p_line_end]
			p_val, p_ok := strconv.parse_int(strings.trim_space(p_line))
			if p_ok do total_points = p_val
		}
	}

	payload_offset := data_idx + line_end + 1

	if is_binary {
		point_size := size_of(PcdBinaryPoint)
		remaining_bytes := len(data) - payload_offset
		n_pts := remaining_bytes / point_size
		if total_points > 0 && total_points < n_pts {
			n_pts = total_points
		}

		reserve(&cloud_out.points, len(cloud_out.points) + n_pts)

		for i in 0 ..< n_pts {
			offset := payload_offset + i * point_size
			raw_bytes := data[offset : offset + point_size]
			bin_pt: PcdBinaryPoint
			dest := transmute([^]u8)&bin_pt
			copy(dest[:point_size], raw_bytes)

			sensor.point_cloud_push(cloud_out, core.Vec3{bin_pt.x, bin_pt.y, bin_pt.z}, bin_pt.intensity, bin_pt.timestamp, bin_pt.ring)
		}
		return true
	} else {
		// Parse ASCII
		payload := text[payload_offset:]
		lines := strings.split_lines(payload)
		defer delete(lines)

		reserve(&cloud_out.points, len(cloud_out.points) + len(lines))

		for line in lines {
			trimmed := strings.trim_space(line)
			if len(trimmed) == 0 do continue

			tokens := strings.fields(trimmed)
			defer delete(tokens)

			if len(tokens) < 3 do continue

			x, ok_x := strconv.parse_f32(tokens[0])
			y, ok_y := strconv.parse_f32(tokens[1])
			z, ok_z := strconv.parse_f32(tokens[2])
			if !ok_x || !ok_y || !ok_z do continue

			intensity := f32(0.0)
			if len(tokens) >= 4 {
				if val, ok_i := strconv.parse_f32(tokens[3]); ok_i {
					intensity = val
				}
			}

			timestamp := f64(0.0)
			if len(tokens) >= 5 {
				if val, ok_t := strconv.parse_f64(tokens[4]); ok_t {
					timestamp = val
				}
			}

			ring := u16(0)
			if len(tokens) >= 6 {
				if val, ok_r := strconv.parse_int(tokens[5]); ok_r {
					ring = u16(val)
				}
			}

			sensor.point_cloud_push(cloud_out, core.Vec3{x, y, z}, intensity, timestamp, ring)
		}
		return true
	}
}
