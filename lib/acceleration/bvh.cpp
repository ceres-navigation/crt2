#include "acceleration/bvh.hpp"

#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"
#include "primitives/aabb.hpp"
#include "utils/vector.hpp"
#include "utils/rotation.hpp"

#include "geometry.hpp"

#include "utils/parallel.hpp"

template <typename Scalar>
BVH<Scalar>::BVH(){
};

template <typename Scalar>
BVH<Scalar>::BVH(AABB<Scalar> bounds){
    this->bounds = bounds;
    this->loaded = false;
};

template <typename Scalar>
BVH<Scalar>::BVH(Triangle<Scalar>* triangles, uint num_triangles) {
    this->tri = triangles;
    this->N = num_triangles;
    this->bvhNode = new BVHNode<Scalar>[N*2];
    this->triIdx = new uint[N];
    this->nodesUsed = 1;
    this->loaded = true;
};

template <typename Scalar>
BVH<Scalar>::~BVH(){
    delete[] this->bvhNode;
    delete[] this->triIdx;
};

template <typename Scalar>
void BVH<Scalar>::init(Triangle<Scalar>* triangles, uint num_triangles) {
    this->tri = triangles;
    this->N = num_triangles;
    this->bvhNode = new BVHNode<Scalar>[N*2];
    this->triIdx = new uint[N];
    this->nodesUsed = 1;
    this->loaded = true;
};

template <typename Scalar>
void BVH<Scalar>::deinit(){
    delete[] this->bvhNode;
    delete[] this->triIdx;
    this->tri = nullptr;
    this->N = 0;
    this->loaded = false;
}

template <typename Scalar>
void BVH<Scalar>::set_parent(Geometry<Scalar>* parent){
    this->parent = parent;
}

template <typename Scalar>
void BVH<Scalar>::UpdateBounds(){   
    Vector3<Scalar> bmin = this->bounds.bmin;
    Vector3<Scalar> bmax = this->bounds.bmax;

    Vector3<Scalar> b1(bmin[0], bmin[1], bmin[2]);
    Vector3<Scalar> b2(bmin[0], bmax[1], bmin[2]);
    Vector3<Scalar> b3(bmax[0], bmax[1], bmin[2]);
    Vector3<Scalar> b4(bmax[0], bmin[1], bmin[2]);

    Vector3<Scalar> t1(bmin[0], bmin[1], bmax[2]);
    Vector3<Scalar> t2(bmin[0], bmax[1], bmax[2]);
    Vector3<Scalar> t3(bmax[0], bmax[1], bmax[2]);
    Vector3<Scalar> t4(bmax[0], bmin[1], bmax[2]);

    // Apply transformation from bdoy to world space:
    transform(b1);
    transform(b2);
    transform(b3);
    transform(b4);
    transform(t1);
    transform(t2);
    transform(t3);
    transform(t4);

    // Reset world bounds:
    this->bounds_world.bmin = Vector3<Scalar>(std::numeric_limits<Scalar>::max());
    this->bounds_world.bmax = Vector3<Scalar>(-std::numeric_limits<Scalar>::max());

    // Grow the bounds based on 
    this->bounds_world.grow(b1);
    this->bounds_world.grow(b2);
    this->bounds_world.grow(b3);
    this->bounds_world.grow(b4);
    this->bounds_world.grow(t1);
    this->bounds_world.grow(t2);
    this->bounds_world.grow(t3);
    this->bounds_world.grow(t4);
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
void BVH<Scalar>::Intersect( Ray<Scalar>& ray, uint tile_number) {
    //TODO REPLACE THIS WITH ORDERED TRAVERSAL:

    // Test the ray against world space pose of the bounding box:
    Scalar dist = intersect_aabb( ray, bounds_world.bmin, bounds_world.bmax );

    // Only proceed with if the ray has not already hit a primitive that is closer:
    if (dist < ray.hit.t){
        // Rotate the ray into the BVH frame:
        Ray<Scalar> backup_ray = ray;
        inverse_transform(ray);

        // Load if needed:
        if (!loaded){
            mutex_t::scoped_lock scoped_lock(init_lock);
            if (!loaded){
                this->parent->load(tile_number);
            }
        }
        this->parent->last_seen_tile_number = tile_number;

        BVHNode<Scalar>& node = bvhNode[0];
        InnerIntersect( ray, node.leftFirst );
        InnerIntersect( ray, node.leftFirst + 1 );

        // Reset the ray:
        backup_ray.t = ray.t;
        backup_ray.hit = ray.hit;
        ray = backup_ray;
    }
};

template <typename Scalar>
void BVH<Scalar>::transform(Vector3<Scalar> &vector){
    vector = this->scale*this->rotation.rotate(vector) + this->position;
};

template <typename Scalar>
void BVH<Scalar>::inverse_transform(Ray<Scalar> &ray){
    ray.origin = this->recip_scale*this->inverse_rotation.rotate(ray.origin - this->position);
    ray.direction = this->inverse_rotation.rotate(ray.direction);
    ray.recip_direction = Vector3<Scalar>(1/ray.direction[0],1/ray.direction[1],1/ray.direction[2]);
};

// Explicitly Instantiate floats and doubles:
template class BVH<float>;
template class BVH<double>;