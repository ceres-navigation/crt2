#include "acceleration/bvh.hpp"

#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"
#include "primitives/aabb.hpp"
#include "vector_math/vector.hpp"

template <typename Scalar>
BVH<Scalar>::BVH(Triangle<Scalar>* triangles, uint num_triangles) {
    this->tri = triangles;
    this->N = num_triangles;
    this->bvhNode = new BVHNode<Scalar>[N*2];
    this->triIdx = new uint[N];
    this->nodesUsed = 1;
};

template <typename Scalar>
void BVH<Scalar>::UpdateNodeBounds( uint nodeIdx ) {
    BVHNode<Scalar>& node = bvhNode[nodeIdx];
    node.aabbMin = Vector3<Scalar>(  std::numeric_limits<Scalar>::max() );
    node.aabbMax = Vector3<Scalar>( -std::numeric_limits<Scalar>::max() );
    for (uint first = node.leftFirst, i = 0; i < node.triCount; i++)
    {
        uint leafTriIdx = triIdx[first + i];
        Triangle<Scalar>& leafTri = tri[leafTriIdx];
        node.aabbMin = min( node.aabbMin, leafTri.vertex0 ),
        node.aabbMin = min( node.aabbMin, leafTri.vertex1 ),
        node.aabbMin = min( node.aabbMin, leafTri.vertex2 ),
        node.aabbMax = max( node.aabbMax, leafTri.vertex0 ),
        node.aabbMax = max( node.aabbMax, leafTri.vertex1 ),
        node.aabbMax = max( node.aabbMax, leafTri.vertex2 );
    }
};

template <typename Scalar>
void BVH<Scalar>::Subdivide( uint nodeIdx ) {
    // terminate recursion
    BVHNode<Scalar>& node = bvhNode[nodeIdx];
    if (node.triCount <= 2){
        return;
    } 

    // determine split axis and position
    Vector3<Scalar> extent = node.aabbMax - node.aabbMin;
    int axis = 0;
    if (extent[1] > extent[0]){
        axis = 1;
    }
    if (extent[2] > extent[axis]){
        axis = 2;
    }
    Scalar splitPos = node.aabbMin[axis] + extent[axis] * (Scalar) 0.5;

    // in-place partition
    int i = node.leftFirst;
    int j = i + node.triCount - 1;
    while (i <= j) {
        if (tri[triIdx[i]].centroid[axis] < splitPos) {
            i++;
        }
        else {
            std::swap( triIdx[i], triIdx[j--] );
        }
    }
    
    // abort split if one of the sides is empty
    uint leftCount = i - node.leftFirst;
    if (leftCount == 0 || leftCount == node.triCount){
        return;
    } 
    // create child nodes
    int leftChildIdx = nodesUsed++;
    int rightChildIdx = nodesUsed++;
    bvhNode[leftChildIdx].leftFirst = node.leftFirst;
    bvhNode[leftChildIdx].triCount = leftCount;
    bvhNode[rightChildIdx].leftFirst = i;
    bvhNode[rightChildIdx].triCount = node.triCount - leftCount;
    node.leftFirst = leftChildIdx;
    node.triCount = 0;
    UpdateNodeBounds( leftChildIdx );
    UpdateNodeBounds( rightChildIdx );

    // recurse
    Subdivide( leftChildIdx );
    Subdivide( rightChildIdx );
};

template <typename Scalar>
void BVH<Scalar>::Build() {
    for (uint i = 0; i < N; i++){
        tri[i].centroid = (tri[i].vertex0 + tri[i].vertex1 + tri[i].vertex2) * (Scalar) 0.3333;
    
        triIdx[i] = i;
    }

    // assign all triangles to root node
    BVHNode<Scalar>& root = bvhNode[0];
    root.leftFirst= 0;
    root.triCount = N;
    UpdateNodeBounds( 0 );

    // subdivide recursively
    Subdivide( 0 );
};

template <typename Scalar>
void BVH<Scalar>::Intersect( Ray<Scalar>& ray, const uint nodeIdx ) {
    BVHNode<Scalar>& node = bvhNode[nodeIdx];
    if (intersect_aabb( ray, node.aabbMin, node.aabbMax ) == std::numeric_limits<Scalar>::max()){
        return;
    }

    if (node.isLeaf()) {
        for (uint i = 0; i < node.triCount; i++ ){
            intersect_triangle( ray, tri[triIdx[node.leftFirst + i]] );
        }
    }
    else {
        Intersect( ray, node.leftFirst );
        Intersect( ray, node.leftFirst + 1 );
    }
};

// Explicitly Instantiate floats and doubles:
template class BVH<float>;
template class BVH<double>;