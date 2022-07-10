#ifndef __BVH_H_
#define __BVH_H_

#include <iostream>
#include <stdint.h>

#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"
#include "primitives/aabb.hpp"
#include "vector_math/vector.hpp"
#include "vector_math/rotation.hpp"

template <typename Scalar>
struct BVHNode {
    Vector3<Scalar> aabbMin, aabbMax;
    uint leftFirst, triCount;
    bool isLeaf() { return triCount > 0; }
};

template<typename Scalar>
struct Bin { 
    AABB<Scalar> bounds;
    int triCount = 0; 
};

template <typename Scalar>
class BVH{
    public:
        Triangle<Scalar>* tri;
        uint N;
        uint nodesUsed;

        // TODO: REMOVE THIS!!!!
        Vector3<Scalar> translation = Vector3<Scalar>(0);
        AABB<Scalar> bounds; // in world space

        BVHNode<Scalar>* bvhNode = nullptr;
        uint* triIdx = nullptr;

        BVH();

        BVH(Triangle<Scalar>* triangles, uint num_triangles);

        ~BVH();

        void SetTranslation( Vector3<Scalar>& translation );

        void UpdateNodeBounds( uint nodeIdx );

        // Functions for Binned BVH building:
        Scalar FindBestSplitPlane( BVHNode<Scalar>& node, int& axis, Scalar& splitPos);
        Scalar CalculateNodeCost( BVHNode<Scalar>& node );
        void Subdivide( uint nodeIdx, int BINS);
        void Build(int BINS=8);

        // Function for ray traversal:
        void Intersect( Ray<Scalar>& ray );
        void IntersectInner( Ray<Scalar>& ray, const uint nodeIdx );
};

#endif