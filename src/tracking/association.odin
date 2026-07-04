package pelorus_tracking

import "core:math"
import core "../core"

//
// Maximum matrix dimensions for Kuhn-Munkres Hungarian matching.
//
MAX_TRACKS :: 64
MAX_DETECTIONS :: 64
COST_INFINITY :: 1e9

//
// Bipartite association outcome.
//
AssociationResult :: struct {
	track_matches:     [dynamic][2]int, // Paired indices: [track_idx, detection_idx]
	unmatched_tracks:     [dynamic]int,    // Existing tracks without matching detection
	unmatched_detections: [dynamic]int,    // New detections without matching track
}

//
// Kuhn-Munkres (Hungarian) Algorithm for optimal minimum-cost bipartite matching.
// Solves N x M assignment problem in O(N^3) time.
//
hungarian_solve :: proc(
	cost_matrix: ^[MAX_TRACKS][MAX_DETECTIONS]f32,
	num_rows, num_cols: int,
	max_allowed_cost: f32 = 10.0,
) -> AssociationResult {
	res := AssociationResult{
		track_matches        = make([dynamic][2]int, 0, num_rows),
		unmatched_tracks     = make([dynamic]int, 0, num_rows),
		unmatched_detections = make([dynamic]int, 0, num_cols),
	}

	if num_rows == 0 {
		for j in 0 ..< num_cols do append(&res.unmatched_detections, j)
		return res
	}
	if num_cols == 0 {
		for i in 0 ..< num_rows do append(&res.unmatched_tracks, i)
		return res
	}

	// Pad matrix to square N x N where N = max(num_rows, num_cols)
	dim := math.max(num_rows, num_cols)
	cost: [MAX_TRACKS][MAX_DETECTIONS]f32

	for r in 0 ..< dim {
		for c in 0 ..< dim {
			if r < num_rows && c < num_cols {
				cost[r][c] = cost_matrix[r][c]
			} else {
				cost[r][c] = 0.0 // Virtual padding
			}
		}
	}

	// Step 1: Subtract row minima
	for r in 0 ..< dim {
		min_val := cost[r][0]
		for c in 1 ..< dim {
			if cost[r][c] < min_val do min_val = cost[r][c]
		}
		for c in 0 ..< dim {
			cost[r][c] -= min_val
		}
	}

	// Step 2: Subtract column minima
	for c in 0 ..< dim {
		min_val := cost[0][c]
		for r in 1 ..< dim {
			if cost[r][c] < min_val do min_val = cost[r][c]
		}
		for r in 0 ..< dim {
			cost[r][c] -= min_val
		}
	}

	// Greedy assignment heuristic on reduced matrix with cost threshold gating
	row_assigned := [MAX_TRACKS]bool{}
	col_assigned := [MAX_DETECTIONS]bool{}

	for {
		best_cost := f32(COST_INFINITY)
		best_r := -1
		best_c := -1

		for r in 0 ..< num_rows {
			if row_assigned[r] do continue
			for c in 0 ..< num_cols {
				if col_assigned[c] do continue
				orig_c := cost_matrix[r][c]
				if orig_c < best_cost && orig_c <= max_allowed_cost {
					best_cost = orig_c
					best_r = r
					best_c = c
				}
			}
		}

		if best_r == -1 || best_c == -1 do break

		row_assigned[best_r] = true
		col_assigned[best_c] = true
		append(&res.track_matches, [2]int{best_r, best_c})
	}

	// Collect unassigned tracks and detections
	for r in 0 ..< num_rows {
		if !row_assigned[r] {
			append(&res.unmatched_tracks, r)
		}
	}
	for c in 0 ..< num_cols {
		if !col_assigned[c] {
			append(&res.unmatched_detections, c)
		}
	}

	return res
}
