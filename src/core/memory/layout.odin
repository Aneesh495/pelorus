package pelorus_memory

//
// Cache-aligned Structure of Arrays (SOA) point cloud buffer.
// Enables sequential SIMD sweeps over coordinate channels without cache thrashing.
//
PointBufferSOA :: struct {
	xs:          []f32,
	ys:          []f32,
	zs:          []f32,
	intensities: []f32,
	timestamps:  []f64,
	rings:       []u16,
	capacity:    int,
	count:       int,
	owns_memory: bool,
}

//
// Initialize PointBufferSOA with pre-allocated capacity.
//
soa_init :: proc(soa: ^PointBufferSOA, capacity: int) {
	soa.xs = make([]f32, capacity)
	soa.ys = make([]f32, capacity)
	soa.zs = make([]f32, capacity)
	soa.intensities = make([]f32, capacity)
	soa.timestamps = make([]f64, capacity)
	soa.rings = make([]u16, capacity)

	soa.capacity = capacity
	soa.count = 0
	soa.owns_memory = true
}

//
// Release backing memory.
//
soa_destroy :: proc(soa: ^PointBufferSOA) {
	if soa.owns_memory {
		if soa.xs != nil do delete(soa.xs)
		if soa.ys != nil do delete(soa.ys)
		if soa.zs != nil do delete(soa.zs)
		if soa.intensities != nil do delete(soa.intensities)
		if soa.timestamps != nil do delete(soa.timestamps)
		if soa.rings != nil do delete(soa.rings)
	}
	soa.xs = nil
	soa.ys = nil
	soa.zs = nil
	soa.intensities = nil
	soa.timestamps = nil
	soa.rings = nil
	soa.capacity = 0
	soa.count = 0
}

//
// Reset count to zero without deallocating backing storage.
//
soa_clear :: proc "contextless" (soa: ^PointBufferSOA) {
	soa.count = 0
}

//
// Append a single point to SOA buffer. Returns false if capacity exceeded.
//
soa_push :: proc "contextless" (
	soa: ^PointBufferSOA,
	x, y, z: f32,
	intensity: f32 = 0.0,
	timestamp: f64 = 0.0,
	ring: u16 = 0,
) -> bool {
	if soa.count >= soa.capacity {
		return false
	}

	idx := soa.count
	soa.xs[idx] = x
	soa.ys[idx] = y
	soa.zs[idx] = z
	soa.intensities[idx] = intensity
	soa.timestamps[idx] = timestamp
	soa.rings[idx] = ring
	soa.count += 1
	return true
}

//
// Get point coordinates at specified index.
//
soa_get_point :: proc "contextless" (soa: ^PointBufferSOA, idx: int) -> (x, y, z: f32) {
	if idx < 0 || idx >= soa.count {
		return 0.0, 0.0, 0.0
	}
	return soa.xs[idx], soa.ys[idx], soa.zs[idx]
}

//
// In-place compaction based on boolean keep mask: retains elements where mask[i] is true.
// Executes in single linear pass O(N) with zero auxiliary memory allocations.
//
soa_filter_inplace :: proc "contextless" (soa: ^PointBufferSOA, keep_mask: []bool) {
	n := soa.count
	if len(keep_mask) < n do return

	write_idx := 0
	for read_idx in 0 ..< n {
		if keep_mask[read_idx] {
			if write_idx != read_idx {
				soa.xs[write_idx] = soa.xs[read_idx]
				soa.ys[write_idx] = soa.ys[read_idx]
				soa.zs[write_idx] = soa.zs[read_idx]
				soa.intensities[write_idx] = soa.intensities[read_idx]
				soa.timestamps[write_idx] = soa.timestamps[read_idx]
				soa.rings[write_idx] = soa.rings[read_idx]
			}
			write_idx += 1
		}
	}

	soa.count = write_idx
}
