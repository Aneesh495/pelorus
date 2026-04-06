package pelorus_memory

import "core:mem"

//
// Monotonic Bump Allocator (Memory Arena).
// Allocates sequentially out of a contiguous memory block with zero fragmentation.
//
Arena :: struct {
	buffer:       []u8,
	offset:       int,
	peak_offset:  int,
	total_allocs: int,
	owns_memory:  bool,
}

//
// Checkpoint marker for scoped scratch rollbacks.
//
ArenaCheckpoint :: struct {
	offset: int,
}

//
// Initialize an Arena with dynamically allocated backing buffer.
//
arena_init :: proc(arena: ^Arena, capacity_bytes: int) {
	arena.buffer = make([]u8, capacity_bytes)
	arena.offset = 0
	arena.peak_offset = 0
	arena.total_allocs = 0
	arena.owns_memory = true
}

//
// Initialize an Arena wrapping an existing static or stack buffer.
//
arena_init_from_buffer :: proc(arena: ^Arena, buffer: []u8) {
	arena.buffer = buffer
	arena.offset = 0
	arena.peak_offset = 0
	arena.total_allocs = 0
	arena.owns_memory = false
}

//
// Release backing memory if owned.
//
arena_destroy :: proc(arena: ^Arena) {
	if arena.owns_memory && arena.buffer != nil {
		delete(arena.buffer)
	}
	arena.buffer = nil
	arena.offset = 0
}

//
// Reset entire arena back to zero offset.
//
arena_reset :: proc(arena: ^Arena) {
	arena.offset = 0
	arena.total_allocs = 0
}

//
// Monotonic bump allocation with alignment support.
//
arena_alloc :: proc(arena: ^Arena, size: int, alignment: int = 8) -> rawptr {
	if size <= 0 do return nil

	align := alignment > 0 ? alignment : 8
	current_ptr := uintptr(&arena.buffer[arena.offset])
	aligned_ptr := (current_ptr + uintptr(align - 1)) & ~uintptr(align - 1)
	padding := int(aligned_ptr - current_ptr)

	new_offset := arena.offset + padding + size
	if new_offset > len(arena.buffer) {
		// Out of memory in arena
		return nil
	}

	arena.offset = new_offset
	if arena.offset > arena.peak_offset {
		arena.peak_offset = arena.offset
	}
	arena.total_allocs += 1

	return rawptr(aligned_ptr)
}

//
// Capture current arena offset as a checkpoint.
//
arena_checkpoint_begin :: proc(arena: ^Arena) -> ArenaCheckpoint {
	return ArenaCheckpoint{offset = arena.offset}
}

//
// Roll back arena offset to previously captured checkpoint.
//
arena_checkpoint_end :: proc(arena: ^Arena, cp: ArenaCheckpoint) {
	if cp.offset <= arena.offset {
		arena.offset = cp.offset
	}
}

//
// Allocator procedure conforming to Odin mem.Allocator signature.
//
arena_allocator_proc :: proc(
	allocator_data: rawptr,
	mode: mem.Allocator_Mode,
	size, alignment: int,
	old_memory: rawptr,
	old_size: int,
	location := #caller_location,
) -> ([]u8, mem.Allocator_Error) {
	arena := (^Arena)(allocator_data)

	switch mode {
	case .Alloc, .Alloc_Non_Zeroed:
		ptr := arena_alloc(arena, size, alignment)
		if ptr == nil {
			return nil, .Out_Of_Memory
		}
		if mode == .Alloc {
			mem.zero(ptr, size)
		}
		return mem.byte_slice(ptr, size), .None

	case .Free:
		// Monotonic arena does not support individual frees
		return nil, .None

	case .Free_All:
		arena_reset(arena)
		return nil, .None

	case .Resize, .Resize_Non_Zeroed:
		// Simple copy-and-reallocate for resizing
		ptr := arena_alloc(arena, size, alignment)
		if ptr == nil {
			return nil, .Out_Of_Memory
		}
		if old_memory != nil && old_size > 0 {
			copy_bytes := size < old_size ? size : old_size
			mem.copy(ptr, old_memory, copy_bytes)
		}
		return mem.byte_slice(ptr, size), .None

	case .Query_Features:
		set := (^mem.Allocator_Mode_Set)(old_memory)
		if set != nil {
			set^ = {.Alloc, .Alloc_Non_Zeroed, .Free_All, .Resize, .Query_Features}
		}
		return nil, .None

	case .Query_Info:
		return nil, .Mode_Not_Implemented
	}

	return nil, .Mode_Not_Implemented
}

//
// Get an Odin mem.Allocator wrapping this Arena.
//
arena_allocator :: proc(arena: ^Arena) -> mem.Allocator {
	return mem.Allocator{
		procedure = arena_allocator_proc,
		data      = rawptr(arena),
	}
}
