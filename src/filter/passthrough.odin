package pelorus_filter

import "core:math"
import core "../core"
import sensor "../sensor"

//
// Passthrough and spatial clipping filter options.
//
PassthroughFilter :: struct {
	min_bounds:     core.Vec3, // Axis-aligned minimum XYZ limits
	max_bounds:     core.Vec3, // Axis-aligned maximum XYZ limits
	min_radius:     f32,       // Minimum radial distance from origin (crops vehicle ego body)
	max_radius:     f32,       // Maximum radial distance (crops distant noise)
	min_azimuth:    f32,       // Minimum horizontal azimuth angle (radians, [-PI, PI])
	max_azimuth:    f32,       // Maximum horizontal azimuth angle (radians, [-PI, PI])
	min_intensity:  f32,
	max_intensity:  f32,
	invert:         bool,      // If true, removes points matching filter criteria (exclusion zone)
}

//
// Default passthrough filter parameters initialized to broad bounds.
//
passthrough_default :: proc() -> PassthroughFilter {
	inf := f32(1e6)
	return PassthroughFilter{
		min_bounds    = core.Vec3{-inf, -inf, -inf},
		max_bounds    = core.Vec3{inf, inf, inf},
		min_radius    = 0.5,
		max_radius    = 150.0,
		min_azimuth   = -f32(math.PI),
		max_azimuth   = f32(math.PI),
		min_intensity = 0.0,
		max_intensity = 255.0,
		invert        = false,
	}
}

//
// Filter PointCloud through spatial clipping rules, appending valid points to cloud_out.
// Returns count of retained points.
//
filter_passthrough :: proc(
	filter: ^PassthroughFilter,
	cloud_out: ^sensor.PointCloud,
	cloud_in: ^sensor.PointCloud,
) -> int {
	n := len(cloud_in.points)
	if n == 0 do return 0

	min_b := filter.min_bounds
	max_b := filter.max_bounds
	min_r_sq := filter.min_radius * filter.min_radius
	max_r_sq := filter.max_radius * filter.max_radius
	min_az := filter.min_azimuth
	max_az := filter.max_azimuth
	min_i := filter.min_intensity
	max_i := filter.max_intensity
	invert := filter.invert

	retained := 0

	for pt in cloud_in.points {
		p := pt.position

		// 1. AABB limits
		inside_box := p.x >= min_b.x && p.x <= max_b.x &&
		              p.y >= min_b.y && p.y <= max_b.y &&
		              p.z >= min_b.z && p.z <= max_b.z

		// 2. Radial limits
		r_sq := p.x * p.x + p.y * p.y + p.z * p.z
		inside_radius := r_sq >= min_r_sq && r_sq <= max_r_sq

		// 3. Azimuthal FOV limits
		azimuth := math.atan2(p.y, p.x)
		inside_azimuth := azimuth >= min_az && azimuth <= max_az

		// 4. Intensity limits
		inside_intensity := pt.intensity >= min_i && pt.intensity <= max_i

		matches := inside_box && inside_radius && inside_azimuth && inside_intensity
		keep := invert ? !matches : matches

		if keep {
			sensor.point_cloud_push(cloud_out, pt.position, pt.intensity, pt.timestamp, pt.ring, pt.normal)
			retained += 1
		}
	}

	return retained
}
