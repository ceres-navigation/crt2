#include "acceleration/bvh.hpp"
#include <cstdlib>

template <typename Scalar>
BVH<Scalar>::BVH(){ }

template <typename Scalar>
BVH<Scalar>::~BVH() { 
    delete [] triangle_index;
    delete [] bvh_node;
};

template <typename Scalar>
void BVH<Scalar>::build(Triangle<Scalar> *triangles, uint32_t num_triangles){
    nodes_used = 1;
    bvh_node = new BVHNode<Scalar>[num_triangles * 2];

    // Initialize array of triangle indices:
    triangle_index = new uint32_t[num_triangles];
    for (uint i = 0; i < num_triangles; i++){
        triangle_index[i] = i;
    }

    // Calculate triangle centroids:
    for (uint i = 0; i < num_triangles; i++){
        triangles[i].centroid = (triangles[i].vertex0 + triangles[i].vertex1 + triangles[i].vertex2) * (Scalar) 0.3333;
    }

    // Initialize root node:
    BVHNode<Scalar> &root = bvh_node[0];
    root.left_first = 0;
    root.num_triangles = num_triangles;

    std::cout << "Number of Triangles: " << num_triangles << "\n";

    // Recursively subdivide and build tree:
    this->update_node_bounds(0, triangles);
    this->subdivide(0, triangles);
};

template <typename Scalar>
AABB<Scalar> BVH<Scalar>::bounds(){
    BVHNode<Scalar>& node = bvh_node[0];
	return { .bmin = node.aabb_min, .bmax = node.aabb_max  };
};

template <typename Scalar> 
void BVH<Scalar>::update_node_bounds( uint node_index, Triangle<Scalar> *triangles){
    BVHNode<Scalar> &node = bvh_node[node_index];

    node.aabb_min = Vector3<Scalar>(std::numeric_limits<Scalar>::max());
    node.aabb_max = Vector3<Scalar>(-std::numeric_limits<Scalar>::max());
    
    for (uint32_t first = node.left_first, i=0; i < node.num_triangles; i++){
        uint32_t leaf_triangle_index = triangle_index[first + i];
        Triangle<Scalar>& leaf_triangle = triangles[leaf_triangle_index];
        std::cout << first + i << "\n";

        // std::cout << "AABB: [" << node.aabb_min[0] << ", " << node.aabb_min[1]  << ", " << node.aabb_min[2];
        // std::cout << ", " << node.aabb_max[0] << ", " << node.aabb_max[1]  << ", " << node.aabb_max[2] << "]\n";

        // std::cout << "leaf_triangle_index: " << leaf_triangle_index << "\n";
        // std::cout << "v0_1 = " << leaf_triangle.vertex0[0] << "\n";
        // std::cout << "v0_1 = " << leaf_triangle.vertex0[1] << "\n";
        // std::cout << "v0_2 = " << leaf_triangle.vertex0[2] << "\n";

        // std::cout << "v0 = ["<<leaf_triangle.vertex0[0] << ", " << leaf_triangle.vertex0[0] << ", " << leaf_triangle.vertex0[0] << "]\n";
        // std::cout << "v1 = ["<<leaf_triangle.vertex1[0] << ", " << leaf_triangle.vertex1[0] << ", " << leaf_triangle.vertex1[0] << "]\n";
        // std::cout << "v2 = ["<<leaf_triangle.vertex2[0] << ", " << leaf_triangle.vertex2[0] << ", " << leaf_triangle.vertex2[0] << "]\n";

        node.aabb_min = min(node.aabb_min, leaf_triangle.vertex0);
        node.aabb_min = min(node.aabb_min, leaf_triangle.vertex1);
        node.aabb_min = min(node.aabb_min, leaf_triangle.vertex2);
        node.aabb_max = max(node.aabb_max, leaf_triangle.vertex0);
        node.aabb_max = max(node.aabb_max, leaf_triangle.vertex1);
        node.aabb_max = max(node.aabb_max, leaf_triangle.vertex2);
    }

    std::cout << "DONE\n";
};

template <typename Scalar> 
void BVH<Scalar>::subdivide(uint32_t node_index, Triangle<Scalar> *triangles){

    BVHNode<Scalar> &node = bvh_node[node_index];
    if (node.num_triangles <= 2){
        return;
    }
    int axis = 0;

    // NAIVE SPACE SPLITTING: ============
    // Replace all of this with binning...
    Vector3<Scalar> extent = node.aabb_max - node.aabb_min;
    if (extent[1] > extent[0]){
        axis = 1;
    }
    if (extent[2] > extent[axis]){
        axis = 2;
    }
    Scalar split_position = node.aabb_min[axis] + extent[axis] * (Scalar) 0.5;
    // ===================================

    uint32_t i = node.left_first;
    uint32_t j = i + node.num_triangles - 1;
    while (i <= j){
        if (triangles[triangle_index[i]].centroid[axis] < split_position){
            i++;
        }
        else {
            std::swap( triangle_index[i], triangle_index[j--]);
        }
    }

    // Exit recursion if side is empty:
    uint32_t left_count = i - node.left_first;
    if (left_count == 0 || left_count == node.num_triangles){
        return;
    }

    // Initialize child nodes:
    uint32_t left_child_index = nodes_used++;
    uint32_t right_child_index = nodes_used++;

    node.left_first = left_child_index;

    bvh_node[left_child_index].left_first = node.left_first;
    bvh_node[left_child_index].num_triangles = left_count;
    bvh_node[right_child_index].left_first = i;
    bvh_node[right_child_index].num_triangles = node.num_triangles - left_count;
    node.num_triangles = 0;

    // Recursively subdivide and build tree:
    this->update_node_bounds(left_child_index, triangles);
    this->update_node_bounds(right_child_index, triangles);
    this->subdivide(left_child_index, triangles);
    this->subdivide(right_child_index, triangles);
};

template <typename Scalar>
void BVH<Scalar>::intersect(Ray<Scalar> &ray, Triangle<Scalar> *triangles){
    BVHNode<Scalar>* node = &bvh_node[0];
    BVHNode<Scalar>* stack[64];
	uint stack_ptr = 0;
	while (1)
	{
		if (node->is_leaf())
		{
			// std::cerr << "Hit BLAS leaf" << std::endl;	
			for (uint i = 0; i < node->num_triangles; i++)
				intersect_triangle( ray, triangles[triangle_index[node->left_first + i]] );
			if (stack_ptr == 0) {
                break; 
            }
            else {
                node = stack[--stack_ptr];
            }
			continue;
		}
		BVHNode<Scalar>* child1 = &bvh_node[node->left_first];
		BVHNode<Scalar>* child2 = &bvh_node[node->left_first + 1];
		Scalar dist1 = intersect_aabb( ray, child1->aabb_min, child1->aabb_max );
		Scalar dist2 = intersect_aabb( ray, child2->aabb_min, child2->aabb_max );

		if (dist1 > dist2) { std::swap( dist1, dist2 ); std::swap( child1, child2 ); }
		if (dist1 == std::numeric_limits<Scalar>::max())
		{
			if (stack_ptr == 0){
                break;
            } 
            else {
                node = stack[--stack_ptr];
            }
		}
		else
		{
			node = child1;
			if (dist2 < std::numeric_limits<Scalar>::max()){
                stack[stack_ptr++] = child2;
            }
		}
	}
};

// Explicitly Instantiate floats and doubles:
template struct BVHNode<float>;
template struct BVHNode<double>;

template class BVH<float>;
template class BVH<double>;