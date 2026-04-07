package pelorus_memory

import "core:mem"

//
// Free-list node embedded within unallocated chunks.
//
PoolNode :: struct {
	next: ^PoolNode,
}

//
// Fixed-size chunk object pool with O(1) alloc and free via embedded free-list.
// Eliminates heap fragmentation for high-frequency objects (voxels, tracking states).
//
ObjectPool :: struct {
	buffer:       []u8,
	chunk_size:   int,
	capacity:     int,
	free_head:    ^PoolNode,
	active_count: int,
	owns_memory:  bool,
}

//
// Initialize ObjectPool with capacity number of elements each chunk_size bytes.
//
pool_init :: proc(pool: ^ObjectPool, chunk_size: int, capacity: int) {
	// Chunk size must be at least large enough to store embedded free-list pointer
	actual_chunk := chunk_size < size_of(PoolNode) ? size_of(PoolNode) : chunk_size
	// Align to 8 bytes
	actual_chunk = (actual_chunk + 7) & ~int(7)

	total_bytes := actual_chunk * capacity
	pool.buffer = make([]u8, total_bytes)
	pool.chunk_size = actual_chunk
	pool.capacity = capacity
	pool.active_count = 0
	pool.owns_memory = true

	// Link all chunks into initial free-list
	pool.free_head = nil
	for i := capacity - 1; i >= 0; i -= 1 {
		node_ptr := (^PoolNode)(&pool.buffer[i * actual_chunk])
		node_ptr.next = pool.free_head
		pool.free_head = node_ptr
	}
}

//
// Destroy object pool and free backing buffer.
//
pool_destroy :: proc(pool: ^ObjectPool) {
	if pool.owns_memory && pool.buffer != nil {
		delete(pool.buffer)
	}
	pool.buffer = nil
	pool.free_head = nil
	pool.active_count = 0
	pool.capacity = 0
}

//
// Allocate a single fixed-size chunk from pool in O(1) time.
// Returns raw pointer or nil if pool exhausted.
//
pool_alloc :: proc(pool: ^ObjectPool) -> rawptr {
	if pool.free_head == nil {
		return nil
	}

	node := pool.free_head
	pool.free_head = node.next
	pool.active_count += 1

	// Zero-initialize allocated chunk
	mem.zero(rawptr(node), pool.chunk_size)
	return rawptr(node)
}

//
// Release a previously allocated chunk back to the free-list in O(1) time.
//
pool_free :: proc(pool: ^ObjectPool, ptr: rawptr) {
	if ptr == nil do return

	// Validate pointer falls within pool buffer boundaries
	ptr_val := uintptr(ptr)
	buf_start := uintptr(&pool.buffer[0])
	buf_end := buf_start + uintptr(len(pool.buffer))

	if ptr_val < buf_start || ptr_val >= buf_end {
		// Pointer outside pool boundary
		return
	}

	node := (^PoolNode)(ptr)
	node.next = pool.free_head
	pool.free_head = node
	pool.active_count -= 1
}

//
// Reset pool: re-links all chunks back into the free-list without reallocating buffer.
//
pool_reset :: proc(pool: ^ObjectPool) {
	pool.free_head = nil
	pool.active_count = 0
	for i := pool.capacity - 1; i >= 0; i -= 1 {
		node_ptr := (^PoolNode)(&pool.buffer[i * pool.chunk_size])
		node_ptr.next = pool.free_head
		pool.free_head = node_ptr
	}
}

//
// Number of free chunks available for allocation.
//
pool_available :: proc "contextless" (pool: ^ObjectPool) -> int {
	return pool.capacity - pool.active_count
}
