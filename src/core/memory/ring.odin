package pelorus_memory

import "core:sync"

//
// Lock-free Single-Producer Single-Consumer (SPSC) circular ring buffer.
// Relies on atomic acquire/release memory semantics and power-of-two capacity masking.
//
SpscRingBuffer :: struct($T: typeid) {
	buffer:      []T,
	mask:        uint,
	head:        uint, // Read cursor, updated by consumer
	tail:        uint, // Write cursor, updated by producer
	owns_memory: bool,
}

//
// Initialize SPSC ring buffer with power-of-two capacity.
// Capacity is rounded up to next power of two if not already power of two.
//
ring_init :: proc(ring: ^SpscRingBuffer($T), requested_capacity: int) {
	cap_val := 4
	for cap_val < requested_capacity {
		cap_val *= 2
	}

	ring.buffer = make([]T, cap_val)
	ring.mask = uint(cap_val - 1)
	ring.head = 0
	ring.tail = 0
	ring.owns_memory = true
}

//
// Release backing memory.
//
ring_destroy :: proc(ring: ^SpscRingBuffer($T)) {
	if ring.owns_memory && ring.buffer != nil {
		delete(ring.buffer)
	}
	ring.buffer = nil
	ring.mask = 0
	ring.head = 0
	ring.tail = 0
}

//
// Total slot capacity of ring buffer.
//
ring_capacity :: proc "contextless" (ring: ^SpscRingBuffer($T)) -> int {
	return int(ring.mask + 1)
}

//
// Number of occupied elements currently available for reading.
//
ring_count :: proc(ring: ^SpscRingBuffer($T)) -> int {
	tail := sync.atomic_load(&ring.tail)
	head := sync.atomic_load(&ring.head)
	return int(tail - head)
}

//
// Number of free slots available for writing.
//
ring_free_slots :: proc(ring: ^SpscRingBuffer($T)) -> int {
	cap_val := int(ring.mask + 1)
	return cap_val - ring_count(ring)
}

//
// Test if ring buffer is empty.
//
ring_is_empty :: proc(ring: ^SpscRingBuffer($T)) -> bool {
	return ring_count(ring) == 0
}

//
// Test if ring buffer is full.
//
ring_is_full :: proc(ring: ^SpscRingBuffer($T)) -> bool {
	return ring_count(ring) >= int(ring.mask + 1)
}

//
// Push a single item into the ring buffer without blocking.
// Returns true on success, false if the buffer is full.
//
ring_push :: proc(ring: ^SpscRingBuffer($T), item: T) -> bool {
	tail := sync.atomic_load(&ring.tail)
	head := sync.atomic_load(&ring.head)

	if (tail - head) > ring.mask {
		// Buffer is full
		return false
	}

	slot := tail & ring.mask
	ring.buffer[slot] = item
	sync.atomic_store_explicit(&ring.tail, tail + 1, .Release)
	return true
}

//
// Pop a single item from the ring buffer without blocking.
// Returns true on success along with the popped item, false if empty.
//
ring_pop :: proc(ring: ^SpscRingBuffer($T)) -> (item: T, ok: bool) {
	head := sync.atomic_load(&ring.head)
	tail := sync.atomic_load_explicit(&ring.tail, .Acquire)

	if head == tail {
		// Buffer is empty
		return {}, false
	}

	slot := head & ring.mask
	item = ring.buffer[slot]
	sync.atomic_store(&ring.head, head + 1)
	return item, true
}

//
// Batch push multiple items into ring buffer.
// Returns number of items successfully written.
//
ring_push_batch :: proc(ring: ^SpscRingBuffer($T), items: []T) -> int {
	tail := sync.atomic_load(&ring.tail)
	head := sync.atomic_load(&ring.head)
	capacity := ring.mask + 1
	available := capacity - (tail - head)

	to_write := uint(len(items))
	if to_write > available {
		to_write = available
	}

	for i in 0 ..< to_write {
		slot := (tail + i) & ring.mask
		ring.buffer[slot] = items[i]
	}

	sync.atomic_store_explicit(&ring.tail, tail + to_write, .Release)
	return int(to_write)
}

//
// Batch pop up to len(out_items) from ring buffer.
// Returns number of items read.
//
ring_pop_batch :: proc(ring: ^SpscRingBuffer($T), out_items: []T) -> int {
	head := sync.atomic_load(&ring.head)
	tail := sync.atomic_load_explicit(&ring.tail, .Acquire)
	available := tail - head

	to_read := uint(len(out_items))
	if to_read > available {
		to_read = available
	}

	for i in 0 ..< to_read {
		slot := (head + i) & ring.mask
		out_items[i] = ring.buffer[slot]
	}

	sync.atomic_store(&ring.head, head + to_read)
	return int(to_read)
}
