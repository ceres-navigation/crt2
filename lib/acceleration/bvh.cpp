#include "acceleration/bvh.hpp"
#include "acceleration/transform_node.hpp"

#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"
#include "primitives/aabb.hpp"
#include "utils/vector.hpp"
#include "utils/rotation.hpp"

template <typename Scalar>
BVH<Scalar>::BVH(){
};

template <typename Scalar>
BVH<Scalar>::BVH(Triangle<Scalar>* triangles, uint num_triangles) {
    this->tri = triangles;
    this->N = num_triangles;
    this->bvhNode = new BVHNode<Scalar>[N*2];
    this->triIdx = new uint[N];
    this->nodesUsed = 1;
};

template <typename Scalar>
BVH<Scalar>::~BVH(){
    delete this->bvhNode;
    delete this->triIdx;
};

template <typename Scalar>
void BVH<Scalar>::set_parent(Geometry<Scalar>* parent){
    this->parent = parent;
}

template <typename Scalar>
void BVH<Scalar>::UpdateBounds(){
    Vector3<Scalar> bmin = this->bounds.bmin;
    Vector3<Scalar> bmax = this->bounds.bmax;
    // Apply all transformations:
    for (int i = 0; i < num_transformations; i++){
        transform_nodes[i].apply(bmin);
        transform_nodes[i].apply(bmax);
    }

    // TODO Just replace the AABB rather than using "grow":
    this->bounds.bmin = bmin;
    this->bounds.bmax = bmax;
};

template <typename Scalar>
void BVH<Scalar>::UpdateNodeBounds( uint nodeIdx ) {
    BVHNode<Scalar>& node = bvhNode[nodeIdx];
    node.aabbMin = Vector3<Scalar>(  std::numeric_limits<Scalar>::max() );
    node.aabbMax = Vector3<Scalar>( -std::numeric_limits<Scalar>::max() );
    for (uint first = node.leftFirst, i = 0; i < node.triCount; i++) {
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
Scalar BVH<Scalar>::FindBestSplitPlane( BVHNode<Scalar>& node, int& axis, Scalar& splitPos){
    int BINS = 8;

    Scalar bestCost = std::numeric_limits<Scalar>::max();
    for (int a = 0; a < 3; a++) {
        Scalar boundsMin = std::numeric_limits<Scalar>::max();
        Scalar boundsMax = -std::numeric_limits<Scalar>::max();
        for (uint i = 0; i < node.triCount; i++) {
            Triangle<Scalar>& triangle = tri[triIdx[node.leftFirst + i]];
            boundsMin = std::min( boundsMin, triangle.centroid[a] );
            boundsMax = std::max( boundsMax, triangle.centroid[a] );
        }
        if (boundsMin == boundsMax) {
            continue;
        }

        // Populate the bins
        Bin<Scalar> bin[BINS];
        Scalar scale = BINS / (boundsMax - boundsMin);
        for (uint i = 0; i < node.triCount; i++) {
            Triangle<Scalar>& triangle = tri[triIdx[node.leftFirst + i]];
            int binIdx = std::min( BINS - 1, (int)((triangle.centroid[a] - boundsMin) * scale) );
            bin[binIdx].triCount++;
            bin[binIdx].bounds.grow( triangle.vertex0 );
            bin[binIdx].bounds.grow( triangle.vertex1 );
            bin[binIdx].bounds.grow( triangle.vertex2 );
        }

        // Gather data for the N-1 planes between the N bins:
        Scalar leftArea[BINS - 1];
        Scalar rightArea[BINS - 1];
        int leftCount[BINS - 1];
        int rightCount[BINS - 1];
        AABB<Scalar> leftBox;
        AABB<Scalar> rightBox;
        int leftSum = 0, rightSum = 0;
        for (int i = 0; i < BINS - 1; i++) {
            leftSum += bin[i].triCount;
            leftCount[i] = leftSum;
            leftBox.grow( bin[i].bounds );
            leftArea[i] = leftBox.area();
            rightSum += bin[BINS - 1 - i].triCount;
            rightCount[BINS - 2 - i] = rightSum;
            rightBox.grow( bin[BINS - 1 - i].bounds );
            rightArea[BINS - 2 - i] = rightBox.area();
        }

        // Calculate SAH cost for the N-1 planes:
        scale = (boundsMax - boundsMin) / BINS;
        for (int i = 0; i < BINS - 1; i++) {
            float planeCost = 
            leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
            if (planeCost < bestCost){
                axis = a;
                splitPos = boundsMin + scale * (i + 1);
                bestCost = planeCost;
            }
        }
    }
    return bestCost;
};

template <typename Scalar>
Scalar BVH<Scalar>::CalculateNodeCost( BVHNode<Scalar>& node ) {
    Vector3<Scalar> e = node.aabbMax - node.aabbMin; // extent of the node
    Scalar surfaceArea = e[0] * e[1] + e[1] * e[2] + e[2] * e[0];
    return node.triCount * surfaceArea;
};

template <typename Scalar>
void BVH<Scalar>::Subdivide( uint nodeIdx, int BINS) {
    // terminate recursion
    BVHNode<Scalar>& node = bvhNode[nodeIdx];
    if (node.triCount <= 2){
        return;
    } 

    // determine split axis and position
    int axis;
    Scalar splitPos;

    // Identify the next best split plane:
    Scalar splitCost = FindBestSplitPlane(node, axis, splitPos);
    Scalar nosplitCost = CalculateNodeCost( node );
    if (splitCost >= nosplitCost){
        return;
    }

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
    Subdivide( leftChildIdx, BINS);
    Subdivide( rightChildIdx, BINS );
};

template <typename Scalar>
void BVH<Scalar>::Build(int BINS) {
    for (uint i = 0; i < N; i++){
        tri[i].centroid = (tri[i].vertex0 + tri[i].vertex1 + tri[i].vertex2) * (Scalar) 0.3333;
    
        triIdx[i] = i;
    }

    // assign all triangles to root node
    BVHNode<Scalar>& root = bvhNode[0];
    root.leftFirst= 0;
    root.triCount = N;
    UpdateNodeBounds( 0 );

    bounds.grow(bvhNode[0].aabbMin);
    bounds.grow(bvhNode[0].aabbMax);

    // subdivide recursively
    Subdivide( 0, BINS);
};

template <typename Scalar>
void BVH<Scalar>::InnerIntersect( Ray<Scalar>& ray, const uint nodeIdx ) {
    // Trace the ray:
    BVHNode<Scalar>& node = bvhNode[nodeIdx];
    if (intersect_aabb( ray, node.aabbMin, node.aabbMax ) == std::numeric_limits<Scalar>::max()){
        return;
    }

    if (node.isLeaf()) {
        for (uint i = 0; i < node.triCount; i++ ){
            auto t = ray.hit.t;
            intersect_triangle( ray, tri[triIdx[node.leftFirst + i]] );
            if (ray.hit.t < t){
                ray.hit.triIdx = triIdx[node.leftFirst + i];
                ray.hit.geometry = parent;
            }
        }
    }
    else {
        InnerIntersect( ray, node.leftFirst );
        InnerIntersect( ray, node.leftFirst + 1 );
    }
};

template <typename Scalar>
void BVH<Scalar>::Intersect( Ray<Scalar>& ray, const uint nodeIdx ) {
    // Apply the transformations:
    Ray<Scalar> backup_ray = ray;
    for (int i = 0; i < num_transformations; i++){
        transform_nodes[i].invert(ray);
    }

    // Trace the ray:
    BVHNode<Scalar>& node = bvhNode[nodeIdx];
    if (intersect_aabb( ray, node.aabbMin, node.aabbMax ) != std::numeric_limits<Scalar>::max()){
        InnerIntersect( ray, node.leftFirst );
        InnerIntersect( ray, node.leftFirst + 1 );
    }

    // Reset the ray:
    backup_ray.t = ray.t;
    backup_ray.hit = ray.hit;
    ray = backup_ray;
};

template <typename Scalar>
AABB<Scalar> BVH<Scalar>::Bounds() {
	BVHNode<Scalar>& node = bvhNode[0];
	return { .bmin = node.aabbMin, .bmax = node.aabbMax  };
}

// Explicitly Instantiate floats and doubles:
template class BVH<float>;
template class BVH<double>;