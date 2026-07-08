package pelorus_tracking

import "core:math"
import core "../core"

//
// Track lifecycle state.
//
TrackState :: enum u8 {
	Tentative,  // New candidate track awaiting confirmation over multiple frames
	Confirmed,  // Active stable track with verified kinematic model
	Coasting,   // Temporarily occluded; propagating via kinematics without measurements
	Terminated, // Target lost or exited sensor field of view
}

//
// Individual multi-object target track.
//
Track :: struct {
	track_id:          u64,
	state:             TrackState,
	filter:            KalmanFilter3D,
	age_frames:        int,
	total_hits:        int,
	consecutive_hits:  int,
	frames_since_meas: int,
	history:           [dynamic]core.Vec3,
}

//
// Multi-Object Tracker (MOT) orchestrating CTRV Kalman filtering,
// Hungarian association, and track lifecycle management.
//
MultiObjectTracker :: struct {
	tracks:               [dynamic]Track,
	next_track_id:        u64,
	min_hits_to_confirm:  int, // Frames required to confirm tentative track (e.g. 3)
	max_coasting_frames:  int, // Frames to coast before terminating (e.g. 5)
	max_association_cost: f32, // Maximum spatial pairing distance threshold (meters)
	owns_memory:          bool,
}

//
// Initialize MultiObjectTracker.
//
tracker_init :: proc(
	tracker: ^MultiObjectTracker,
	min_hits_to_confirm: int = 3,
	max_coasting_frames: int = 5,
	max_cost: f32 = 4.0,
) {
	tracker.tracks = make([dynamic]Track, 0, 64)
	tracker.next_track_id = 1
	tracker.min_hits_to_confirm = min_hits_to_confirm
	tracker.max_coasting_frames = max_coasting_frames
	tracker.max_association_cost = max_cost
	tracker.owns_memory = true
}

//
// Destroy tracker and release track trajectory histories.
//
tracker_destroy :: proc(tracker: ^MultiObjectTracker) {
	if tracker.owns_memory {
		for i in 0 ..< len(tracker.tracks) {
			delete(tracker.tracks[i].history)
		}
		delete(tracker.tracks)
	}
	tracker.tracks = nil
}

//
// Step the multi-object tracker: predicts tracks, associates with new detections,
// updates Kalman filters, and updates track lifecycle states.
//
tracker_step :: proc(
	tracker: ^MultiObjectTracker,
	detections: []BoundingBox3D,
	dt: f32,
) {
	num_tracks := len(tracker.tracks)
	num_dets := len(detections)

	// 1. Predict all existing active tracks forward by dt
	for i in 0 ..< num_tracks {
		kalman3d_predict(&tracker.tracks[i].filter, dt)
		tracker.tracks[i].age_frames += 1
		tracker.tracks[i].frames_since_meas += 1

		if tracker.tracks[i].state == .Confirmed && tracker.tracks[i].frames_since_meas > 0 {
			tracker.tracks[i].state = .Coasting
		}
	}

	// 2. Build cost matrix between active tracks and new detections
	cost_matrix: [MAX_TRACKS][MAX_DETECTIONS]f32
	clamped_tracks := math.min(num_tracks, MAX_TRACKS)
	clamped_dets := math.min(num_dets, MAX_DETECTIONS)

	for r in 0 ..< clamped_tracks {
		track_pos := core.Vec3{
			tracker.tracks[r].filter.state[0],
			tracker.tracks[r].filter.state[1],
			tracker.tracks[r].filter.state[2],
		}

		for c in 0 ..< clamped_dets {
			det_pos := detections[c].center
			dist := core.vec3_distance(track_pos, det_pos)

			// Combine Euclidean distance with dimensional similarity
			dim_diff := core.vec3_distance(tracker.tracks[r].filter.dimensions, detections[c].size)
			cost_matrix[r][c] = dist + 0.3 * dim_diff
		}
	}

	// 3. Solve global Hungarian assignment
	assoc := hungarian_solve(&cost_matrix, clamped_tracks, clamped_dets, tracker.max_association_cost)
	defer {
		delete(assoc.track_matches)
		delete(assoc.unmatched_tracks)
		delete(assoc.unmatched_detections)
	}

	// 4. Update matched tracks with their paired detection
	for pair in assoc.track_matches {
		t_idx := pair[0]
		d_idx := pair[1]

		det := detections[d_idx]
		kalman3d_update(&tracker.tracks[t_idx].filter, det.center, det.yaw, det.size)

		tracker.tracks[t_idx].total_hits += 1
		tracker.tracks[t_idx].consecutive_hits += 1
		tracker.tracks[t_idx].frames_since_meas = 0

		// Lifecycle state transition: Tentative -> Confirmed
		if tracker.tracks[t_idx].state == .Tentative && tracker.tracks[t_idx].consecutive_hits >= tracker.min_hits_to_confirm {
			tracker.tracks[t_idx].state = .Confirmed
		} else if tracker.tracks[t_idx].state == .Coasting {
			tracker.tracks[t_idx].state = .Confirmed
		}

		// Append to position history
		curr_pos := core.Vec3{
			tracker.tracks[t_idx].filter.state[0],
			tracker.tracks[t_idx].filter.state[1],
			tracker.tracks[t_idx].filter.state[2],
		}
		append(&tracker.tracks[t_idx].history, curr_pos)
		if len(tracker.tracks[t_idx].history) > 30 {
			ordered_remove(&tracker.tracks[t_idx].history, 0)
		}
	}

	// 5. Reset consecutive hits for unmatched tracks
	for t_idx in assoc.unmatched_tracks {
		tracker.tracks[t_idx].consecutive_hits = 0
		if tracker.tracks[t_idx].frames_since_meas > tracker.max_coasting_frames {
			tracker.tracks[t_idx].state = .Terminated
		}
	}

	// 6. Spawn new tentative tracks from unmatched detections
	for d_idx in assoc.unmatched_detections {
		det := detections[d_idx]

		var_kf: KalmanFilter3D
		kalman3d_init(&var_kf, det.center, det.yaw, det.size)

		hist := make([dynamic]core.Vec3, 0, 32)
		append(&hist, det.center)

		new_track := Track{
			track_id          = tracker.next_track_id,
			state             = .Tentative,
			filter            = var_kf,
			age_frames        = 1,
			total_hits        = 1,
			consecutive_hits  = 1,
			frames_since_meas = 0,
			history           = hist,
		}
		tracker.next_track_id += 1
		append(&tracker.tracks, new_track)
	}

	// 7. Prune terminated tracks
	write_idx := 0
	for read_idx in 0 ..< len(tracker.tracks) {
		if tracker.tracks[read_idx].state != .Terminated {
			if write_idx != read_idx {
				tracker.tracks[write_idx] = tracker.tracks[read_idx]
			}
			write_idx += 1
		} else {
			delete(tracker.tracks[read_idx].history)
		}
	}
	resize(&tracker.tracks, write_idx)
}
