package pelorus_mapping

//
// 64-bit Morton Z-order curve spatial hashing.
// Interleaves the bits of 3D integer coordinates (x, y, z) into a single 64-bit scalar.
// Guarantees spatial locality: contiguous voxels in 3D Euclidean space map to clustered
// memory indices, maximizing CPU L1/L2 data cache hit rates.
//

//
// Spreads 21 bits of integer x across 63 bits (placing 2 zero bits between each bit).
// Uses branchless parallel bit manipulation masks.
//
morton_part1by2 :: proc "contextless" (n_in: u64) -> u64 {
	n := n_in & 0x1fffff // Keep lower 21 bits (range: 0 to 2,097,151)
	n = (n | (n << 32)) & 0x1f00000000ffff
	n = (n | (n << 16)) & 0x1f0000ff0000ff
	n = (n | (n << 8))  & 0x100f00f00f00f00f
	n = (n | (n << 4))  & 0x10c30c30c30c30c3
	n = (n | (n << 2))  & 0x1249249249249249
	return n
}

//
// Inverses bit dilation: extracts 21 bits from 63-bit interleaved integer.
//
morton_compact1by2 :: proc "contextless" (n_in: u64) -> u64 {
	n := n_in & 0x1249249249249249
	n = (n ~ (n >> 2))  & 0x10c30c30c30c30c3
	n = (n ~ (n >> 4))  & 0x100f00f00f00f00f
	n = (n ~ (n >> 8))  & 0x1f0000ff0000ff
	n = (n ~ (n >> 16)) & 0x1f00000000ffff
	n = (n ~ (n >> 32)) & 0x1fffff
	return n
}

//
// Encode signed 3D discrete grid coordinates (x, y, z) into 64-bit Morton Z-order key.
// Offset by 2^20 (1,048,576) to map negative coordinate ranges to positive integers.
//
MORTON_COORD_OFFSET :: 1048576

morton_encode_3d :: proc "contextless" (x, y, z: i32) -> u64 {
	ux := u64(x + MORTON_COORD_OFFSET) & 0x1fffff
	uy := u64(y + MORTON_COORD_OFFSET) & 0x1fffff
	uz := u64(z + MORTON_COORD_OFFSET) & 0x1fffff

	return (morton_part1by2(uz) << 2) | (morton_part1by2(uy) << 1) | morton_part1by2(ux)
}

//
// Decode 64-bit Morton key back into signed 3D grid coordinates (x, y, z).
//
morton_decode_3d :: proc "contextless" (code: u64) -> (x, y, z: i32) {
	rx := morton_compact1by2(code)
	ry := morton_compact1by2(code >> 1)
	rz := morton_compact1by2(code >> 2)

	x = i32(rx) - MORTON_COORD_OFFSET
	y = i32(ry) - MORTON_COORD_OFFSET
	z = i32(rz) - MORTON_COORD_OFFSET
	return
}
