#ifndef __BVH_H_
#define __BVH_H_

#include <iostream>
#include <stdint.h>

#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"
#include "primitives/aabb.hpp"
#include "vector_math/vector.hpp"

template <typename Scalar>
struct BVHNode {
    Vector3<Scalar> aabbMin, aabbMax;
    uint leftFirst, triCount;
    bool isLeaf() { return triCount > 0; }
};

template <typename Scalar>
class BVH{
    public:
        Triangle<Scalar>* tri;
        uint N;
        uint nodesUsed;

        BVHNode<Scalar>* bvhNode;
        uint* triIdx;

        BVH(Triangle<Scalar>* triangles, uint num_triangles);

        void UpdateNodeBounds( uint nodeIdx );

        void Subdivide( uint nodeIdx );

        void Build();

        void Intersect( Ray<Scalar>& ray, const uint nodeIdx );
};

#endif