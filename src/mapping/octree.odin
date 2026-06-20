package pelorus_mapping

import "core:math"
import core "../core"
import sensor "../sensor"

//
// Octree node representing an axis-aligned spatial bounding octant.
//
OctreeNode :: struct {
	bounds:         core.AABB,
	children:       [8]int, // Indices into node pool (-1 if child does not exist)
	point_indices:  [dynamic]int,
	is_leaf:        bool,
	depth:          int,
}

//
// Dynamic Spatial Octree supporting hierarchical radius searches and frustum culling.
//
Octree :: struct {
	nodes:         [dynamic]OctreeNode,
	points:        []core.Vec3,
	max_depth:     int,
	leaf_capacity: int,
	owns_memory:   bool,
}

//
// Initialize Octree with bounding box and parameters.
//
octree_init :: proc(tree: ^Octree, bounds: core.AABB, max_depth: int = 8, leaf_capacity: int = 32) {
	tree.nodes = make([dynamic]OctreeNode, 0, 1024)
	tree.max_depth = max_depth
	tree.leaf_capacity = leaf_capacity
	tree.owns_memory = true

	root_node := OctreeNode{
		bounds        = bounds,
		children      = [8]int{-1, -1, -1, -1, -1, -1, -1, -1},
		point_indices = make([dynamic]int, 0, leaf_capacity),
		is_leaf       = true,
		depth         = 0,
	}
	append(&tree.nodes, root_node)
}

//
// Destroy Octree and release node buffers.
//
octree_destroy :: proc(tree: ^Octree) {
	if tree.owns_memory {
		for i in 0 ..< len(tree.nodes) {
			delete(tree.nodes[i].point_indices)
		}
		delete(tree.nodes)
	}
	tree.nodes = nil
}

//
// Determine which of the 8 child octants a point belongs to relative to center.
// Octant bitmask: bit 0: X >= center.x, bit 1: Y >= center.y, bit 2: Z >= center.z.
//
octree_child_index :: proc "contextless" (center, p: core.Vec3) -> int {
	idx := 0
	if p.x >= center.x do idx |= 1
	if p.y >= center.y do idx |= 2
	if p.z >= center.z do idx |= 4
	return idx
}

//
// Subdivide a leaf node into 8 child octants.
//
octree_subdivide :: proc(tree: ^Octree, node_idx: int) {
	node := &tree.nodes[node_idx]
	center := core.aabb_center(node.bounds)
	min_p := node.bounds.min_pt
	max_p := node.bounds.max_pt
	next_depth := node.depth + 1

	for oct := 0; oct < 8; oct += 1 {
		child_min := core.Vec3{
			(oct & 1) != 0 ? center.x : min_p.x,
			(oct & 2) != 0 ? center.y : min_p.y,
			(oct & 4) != 0 ? center.z : min_p.z,
		}
		child_max := core.Vec3{
			(oct & 1) != 0 ? max_p.x : center.x,
			(oct & 2) != 0 ? max_p.y : center.y,
			(oct & 4) != 0 ? max_p.z : center.z,
		}

		child_node := OctreeNode{
			bounds        = core.aabb_make(child_min, child_max),
			children      = [8]int{-1, -1, -1, -1, -1, -1, -1, -1},
			point_indices = make([dynamic]int, 0, tree.leaf_capacity),
			is_leaf       = true,
			depth         = next_depth,
		}

		child_idx := len(tree.nodes)
		append(&tree.nodes, child_node)
		tree.nodes[node_idx].children[oct] = child_idx
	}

	tree.nodes[node_idx].is_leaf = false

	// Re-distribute existing points to children
	for pt_idx in tree.nodes[node_idx].point_indices {
		p := tree.points[pt_idx]
		oct := octree_child_index(center, p)
		child_idx := tree.nodes[node_idx].children[oct]
		append(&tree.nodes[child_idx].point_indices, pt_idx)
	}

	clear(&tree.nodes[node_idx].point_indices)
}

//
// Insert point index into octree node recursively.
//
octree_insert :: proc(tree: ^Octree, node_idx: int, pt_idx: int) {
	node := &tree.nodes[node_idx]
	p := tree.points[pt_idx]

	if !core.aabb_contains_point(node.bounds, p) {
		return
	}

	if node.is_leaf {
		append(&node.point_indices, pt_idx)
		if len(node.point_indices) > tree.leaf_capacity && node.depth < tree.max_depth {
			octree_subdivide(tree, node_idx)
		}
		return
	}

	center := core.aabb_center(node.bounds)
	oct := octree_child_index(center, p)
	child_idx := node.children[oct]
	if child_idx != -1 {
		octree_insert(tree, child_idx, pt_idx)
	}
}

//
// Build Octree over an array of 3D points.
//
octree_build :: proc(tree: ^Octree, points: []core.Vec3) {
	tree.points = points
	for i in 0 ..< len(points) {
		octree_insert(tree, 0, i)
	}
}

//
// Query points within search sphere of given radius around query center.
//
octree_query_radius :: proc(
	tree: ^Octree,
	node_idx: int,
	center: core.Vec3,
	radius: f32,
	results: ^[dynamic]int,
) {
	node := &tree.nodes[node_idx]

	// Check if search sphere intersects node AABB
	half_ext := core.aabb_half_extents(node.bounds)
	box_center := core.aabb_center(node.bounds)
	delta := center - box_center
	closest_x := math.clamp(delta.x, -half_ext.x, half_ext.x)
	closest_y := math.clamp(delta.y, -half_ext.y, half_ext.y)
	closest_z := math.clamp(delta.z, -half_ext.z, half_ext.z)
	closest_pt := box_center + core.Vec3{closest_x, closest_y, closest_z}

	if core.vec3_distance_sq(center, closest_pt) > (radius * radius) {
		return // Box is entirely outside search radius
	}

	if node.is_leaf {
		rad_sq := radius * radius
		for pt_idx in node.point_indices {
			if core.vec3_distance_sq(center, tree.points[pt_idx]) <= rad_sq {
				append(results, pt_idx)
			}
		}
		return
	}

	for oct in 0 ..< 8 {
		child_idx := node.children[oct]
		if child_idx != -1 {
			octree_query_radius(tree, child_idx, center, radius, results)
		}
	}
}
