package pelorus_tests

import "core:testing"
import mem "../src/core/memory"

@(test)
test_arena_allocator :: proc(t: ^testing.T) {
	arena: mem.Arena
	mem.arena_init(&arena, 4096)
	defer mem.arena_destroy(&arena)

	ptr1 := mem.arena_alloc(&arena, 64, 8)
	testing.expect(t, ptr1 != nil, "Arena alloc 1 failed")
	testing.expect_value(t, arena.total_allocs, 1)

	// Checkpoint rollback test
	cp := mem.arena_checkpoint_begin(&arena)
	ptr2 := mem.arena_alloc(&arena, 128, 8)
	testing.expect(t, ptr2 != nil, "Arena alloc 2 failed")
	testing.expect_value(t, arena.total_allocs, 2)

	mem.arena_checkpoint_end(&arena, cp)
	testing.expect_value(t, arena.offset, cp.offset)

	// Full reset test
	mem.arena_reset(&arena)
	testing.expect_value(t, arena.offset, 0)
}

@(test)
test_object_pool :: proc(t: ^testing.T) {
	pool: mem.ObjectPool
	mem.pool_init(&pool, 32, 4)
	defer mem.pool_destroy(&pool)

	testing.expect_value(t, mem.pool_available(&pool), 4)

	c1 := mem.pool_alloc(&pool)
	c2 := mem.pool_alloc(&pool)
	c3 := mem.pool_alloc(&pool)
	c4 := mem.pool_alloc(&pool)

	testing.expect(t, c1 != nil, "Pool alloc 1 failed")
	testing.expect(t, c2 != nil, "Pool alloc 2 failed")
	testing.expect(t, c3 != nil, "Pool alloc 3 failed")
	testing.expect(t, c4 != nil, "Pool alloc 4 failed")
	testing.expect_value(t, mem.pool_available(&pool), 0)

	// Allocating past capacity returns nil
	c5 := mem.pool_alloc(&pool)
	testing.expect(t, c5 == nil, "Exhausted pool should return nil")

	// Free and re-alloc
	mem.pool_free(&pool, c2)
	testing.expect_value(t, mem.pool_available(&pool), 1)

	re_c2 := mem.pool_alloc(&pool)
	testing.expect_value(t, re_c2, c2)
}

@(test)
test_spsc_ring_buffer :: proc(t: ^testing.T) {
	ring: mem.SpscRingBuffer(int)
	mem.ring_init(&ring, 8)
	defer mem.ring_destroy(&ring)

	testing.expect(t, mem.ring_is_empty(&ring), "Initial ring must be empty")

	// Push 3 items
	testing.expect(t, mem.ring_push(&ring, 10), "Push 10 failed")
	testing.expect(t, mem.ring_push(&ring, 20), "Push 20 failed")
	testing.expect(t, mem.ring_push(&ring, 30), "Push 30 failed")
	testing.expect_value(t, mem.ring_count(&ring), 3)

	// Pop 1 item
	val1, ok1 := mem.ring_pop(&ring)
	testing.expect(t, ok1, "Pop 1 failed")
	testing.expect_value(t, val1, 10)
	testing.expect_value(t, mem.ring_count(&ring), 2)

	// Batch push
	batch_in := [3]int{40, 50, 60}
	pushed := mem.ring_push_batch(&ring, batch_in[:])
	testing.expect_value(t, pushed, 3)

	// Batch pop
	batch_out: [4]int
	popped := mem.ring_pop_batch(&ring, batch_out[:])
	testing.expect_value(t, popped, 4)
	testing.expect_value(t, batch_out[0], 20)
	testing.expect_value(t, batch_out[1], 30)
	testing.expect_value(t, batch_out[2], 40)
	testing.expect_value(t, batch_out[3], 50)
}

@(test)
test_soa_point_buffer :: proc(t: ^testing.T) {
	soa: mem.PointBufferSOA
	mem.soa_init(&soa, 16)
	defer mem.soa_destroy(&soa)

	mem.soa_push(&soa, 1.0, 2.0, 3.0, 0.5, 100.0, 1)
	mem.soa_push(&soa, 4.0, 5.0, 6.0, 0.8, 101.0, 2)
	mem.soa_push(&soa, 7.0, 8.0, 9.0, 0.2, 102.0, 3)
	testing.expect_value(t, soa.count, 3)

	x, y, z := mem.soa_get_point(&soa, 1)
	testing.expect_value(t, x, 4.0)
	testing.expect_value(t, y, 5.0)
	testing.expect_value(t, z, 6.0)

	// Filter in-place: keep indices 0 and 2
	mask := [3]bool{true, false, true}
	mem.soa_filter_inplace(&soa, mask[:])
	testing.expect_value(t, soa.count, 2)

	x1, y1, z1 := mem.soa_get_point(&soa, 1)
	testing.expect_value(t, x1, 7.0)
	testing.expect_value(t, y1, 8.0)
	testing.expect_value(t, z1, 9.0)
}
