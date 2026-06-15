package pelorus_mapping

import "core:math"
import core "../core"

//
// Dimensions of an individual sparse voxel block: 8x8x8 = 512 voxels.
//
VOXEL_BLOCK_SIZE :: 8
VOXELS_PER_BLOCK :: 512

//
// Individual voxel node storing Truncated Signed Distance Field (TSDF) values.
//
VoxelNode :: struct {
	sdf:     f32, // Truncated signed distance to nearest surface (negative inside, positive outside)
	weight:  f32, // Accumulated observation weight
	color_r: u8,
	color_g: u8,
	color_b: u8,
	flags:   u8,  // Bit 0: observed, Bit 1: surface boundary
}

//
// Dense contiguous memory block for a local 8x8x8 cubic volume.
//
VoxelBlock :: struct {
	coord_x: i32, // Block spatial grid coordinates
	coord_y: i32,
	coord_z: i32,
	voxels:  [VOXELS_PER_BLOCK]VoxelNode,
}

//
// Robin Hood hash table bucket.
//
HashBucket :: struct {
	key:         u64,  // Morton or spatial hash key
	block_index: int,  // Index into blocks array (-1 if empty)
	psl:         u16,  // Probe Sequence Length for Robin Hood displacement
	occupied:    bool,
}

//
// Sparse Hierarchical Voxel Hash Table with Robin Hood open-addressing probing.
// Scales dynamically to arbitrary world bounds without allocating empty space.
//
VoxelHashTable :: struct {
	voxel_size:   f32, // Physical edge length of a single voxel (meters, e.g. 0.05)
	inv_voxel:    f32,
	block_size_m: f32, // Physical edge length of a block: 8 * voxel_size
	inv_block_m:  f32,
	buckets:      []HashBucket,
	bucket_mask:  u64,
	blocks:       [dynamic]VoxelBlock,
	free_blocks:  [dynamic]int,
	active_blocks: int,
	owns_memory:  bool,
}

//
// Compute local linear voxel index in [0..511] from block-relative coordinate (lx, ly, lz) in [0..7].
//
voxel_local_index :: proc "contextless" (lx, ly, lz: i32) -> int {
	return int((lz << 6) | (ly << 3) | lx)
}

//
// Initialize VoxelHashTable with physical voxel resolution and capacity.
//
voxel_hash_init :: proc(table: ^VoxelHashTable, voxel_size: f32 = 0.05, initial_blocks: int = 1024) {
	v_size := voxel_size > 0.001 ? voxel_size : 0.05
	table.voxel_size = v_size
	table.inv_voxel = 1.0 / v_size
	table.block_size_m = v_size * f32(VOXEL_BLOCK_SIZE)
	table.inv_block_m = 1.0 / table.block_size_m

	bucket_cap := 1024
	for bucket_cap < initial_blocks * 2 {
		bucket_cap *= 2
	}

	table.buckets = make([]HashBucket, bucket_cap)
	table.bucket_mask = u64(bucket_cap - 1)
	table.blocks = make([dynamic]VoxelBlock, 0, initial_blocks)
	table.free_blocks = make([dynamic]int, 0, 256)
	table.active_blocks = 0
	table.owns_memory = true

	for i in 0 ..< bucket_cap {
		table.buckets[i].occupied = false
		table.buckets[i].block_index = -1
		table.buckets[i].psl = 0
	}
}

//
// Destroy VoxelHashTable and free all allocated blocks.
//
voxel_hash_destroy :: proc(table: ^VoxelHashTable) {
	if table.owns_memory {
		if table.buckets != nil do delete(table.buckets)
		delete(table.blocks)
		delete(table.free_blocks)
	}
	table.buckets = nil
	table.active_blocks = 0
}

//
// Lookup voxel block by 3D block coordinate (bx, by, bz).
// Returns pointer to block or nil if not allocated.
//
voxel_hash_find_block :: proc(table: ^VoxelHashTable, bx, by, bz: i32) -> ^VoxelBlock {
	key := morton_encode_3d(bx, by, bz)
	mask := table.bucket_mask
	slot := key & mask
	dist: u16 = 0

	for {
		b := &table.buckets[slot]
		if !b.occupied || dist > b.psl {
			return nil
		}
		if b.key == key {
			return &table.blocks[b.block_index]
		}
		slot = (slot + 1) & mask
		dist += 1
	}
}

//
// Allocate or fetch voxel block for 3D block coordinate (bx, by, bz).
// Uses Robin Hood probing to insert new bucket entry.
//
voxel_hash_get_or_create_block :: proc(table: ^VoxelHashTable, bx, by, bz: i32) -> ^VoxelBlock {
	key := morton_encode_3d(bx, by, bz)
	mask := table.bucket_mask
	slot := key & mask
	dist: u16 = 0

	// Check if block already exists
	for {
		b := &table.buckets[slot]
		if !b.occupied || dist > b.psl {
			break
		}
		if b.key == key {
			return &table.blocks[b.block_index]
		}
		slot = (slot + 1) & mask
		dist += 1
	}

	// Allocate block from pool or append
	block_idx: int
	if len(table.free_blocks) > 0 {
		block_idx = pop(&table.free_blocks)
		table.blocks[block_idx].coord_x = bx
		table.blocks[block_idx].coord_y = by
		table.blocks[block_idx].coord_z = bz
		// Reset voxels
		for v in 0 ..< VOXELS_PER_BLOCK {
			table.blocks[block_idx].voxels[v] = VoxelNode{
				sdf     = 1.0,
				weight  = 0.0,
				color_r = 255,
				color_g = 255,
				color_b = 255,
				flags   = 0,
			}
		}
	} else {
		block_idx = len(table.blocks)
		new_block := VoxelBlock{
			coord_x = bx,
			coord_y = by,
			coord_z = bz,
		}
		for v in 0 ..< VOXELS_PER_BLOCK {
			new_block.voxels[v] = VoxelNode{
				sdf     = 1.0,
				weight  = 0.0,
				color_r = 255,
				color_g = 255,
				color_b = 255,
				flags   = 0,
			}
		}
		append(&table.blocks, new_block)
	}

	table.active_blocks += 1

	// Robin Hood insertion
	curr_entry := HashBucket{
		key         = key,
		block_index = block_idx,
		psl         = 0,
		occupied    = true,
	}

	ins_slot := key & mask
	for {
		b := &table.buckets[ins_slot]
		if !b.occupied {
			b^ = curr_entry
			return &table.blocks[block_idx]
		}

		// Robin Hood swap: if incoming entry has higher probe distance, steal bucket
		if curr_entry.psl > b.psl {
			tmp := b^
			b^ = curr_entry
			curr_entry = tmp
		}

		curr_entry.psl += 1
		ins_slot = (ins_slot + 1) & mask
	}
}

//
// Fetch VoxelNode at continuous metric 3D point (px, py, pz).
// Returns pointer to VoxelNode and success flag.
//
voxel_hash_get_voxel :: proc(table: ^VoxelHashTable, p: core.Vec3, allocate_if_missing: bool = false) -> (^VoxelNode, bool) {
	// Discrete voxel coordinates
	vx := i32(math.floor(p.x * table.inv_voxel))
	vy := i32(math.floor(p.y * table.inv_voxel))
	vz := i32(math.floor(p.z * table.inv_voxel))

	// Block coordinates
	bx := vx >> 3
	by := vy >> 3
	bz := vz >> 3

	// Local voxel offset within block in [0..7]
	lx := vx & 7
	ly := vy & 7
	lz := vz & 7

	var_block: ^VoxelBlock
	if allocate_if_missing {
		var_block = voxel_hash_get_or_create_block(table, bx, by, bz)
	} else {
		var_block = voxel_hash_find_block(table, bx, by, bz)
	}

	if var_block == nil {
		return nil, false
	}

	local_idx := voxel_local_index(lx, ly, lz)
	return &var_block.voxels[local_idx], true
}
