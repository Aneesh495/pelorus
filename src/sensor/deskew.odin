package pelorus_sensor

import "core:math"
import core "../core"

//
// Motion deskewing diagnostic statistics.
//
DeskewStats :: struct {
	points_deskewed:   int,
	max_displacement:  f32, // Maximum spatial displacement corrected (meters)
	mean_displacement: f32, // Mean spatial displacement corrected (meters)
	valid_fraction:    f32,
}

//
// Motion deskewing algorithm: un-warps spinning/solid-state LiDAR point cloud
// by projecting each laser firing onto a single rigid reference timestamp using continuous SE(3) poses.
//
deskew_point_cloud :: proc(
	cloud_out: ^PointCloud,
	cloud_in: ^PointCloud,
	traj: ^core.TrajectoryBuffer,
	ref_timestamp: f64,
) -> DeskewStats {
	stats := DeskewStats{}
	n := len(cloud_in.points)
	if n == 0 do return stats

	ref_pose, ok_ref := core.trajectory_buffer_query(traj, ref_timestamp)
	if !ok_ref {
		// Reference pose not available, copy unchanged
		for pt in cloud_in.points {
			point_cloud_push(cloud_out, pt.position, pt.intensity, pt.timestamp, pt.ring, pt.normal)
		}
		return stats
	}

	inv_ref_pose := core.transform3d_inverse(ref_pose)

	total_disp: f32 = 0.0
	max_disp: f32 = 0.0
	valid_count := 0

	for pt in cloud_in.points {
		pt_time := pt.timestamp

		// Query sensor SE(3) pose at exact point timestamp
		pt_pose, ok_pt := core.trajectory_buffer_query(traj, pt_time)
		if !ok_pt {
			// Pose lookup failed, keep original point position
			point_cloud_push(cloud_out, pt.position, pt.intensity, pt.timestamp, pt.ring, pt.normal)
			continue
		}

		// Relative transform from firing time to reference frame:
		// delta_T = T_{ref}^{-1} * T_{pt}
		delta_tf := core.transform3d_mul(inv_ref_pose, pt_pose)

		// Un-warp point into reference frame
		deskewed_pos := core.transform3d_apply_point(delta_tf, pt.position)

		disp := core.vec3_distance(deskewed_pos, pt.position)
		total_disp += disp
		if disp > max_disp {
			max_disp = disp
		}

		// Rotate surface normal if present
		deskewed_norm := core.transform3d_apply_vector(delta_tf, pt.normal)

		point_cloud_push(cloud_out, deskewed_pos, pt.intensity, pt.timestamp, pt.ring, deskewed_norm)
		valid_count += 1
	}

	stats.points_deskewed = valid_count
	stats.max_displacement = max_disp
	stats.mean_displacement = valid_count > 0 ? (total_disp / f32(valid_count)) : 0.0
	stats.valid_fraction = n > 0 ? (f32(valid_count) / f32(n)) : 0.0

	return stats
}
