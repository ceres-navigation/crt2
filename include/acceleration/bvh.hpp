#ifndef __BVH_H_
#define __BVH_H_

#include <iostream>
#include <stdint.h>

#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"
#include "primitives/aabb.hpp"
#include "utils/vector.hpp"
#include "utils/rotation.hpp"

#include "utils/parallel.hpp"

// Forward declaration of Geometry class:
template <typename Scalar>
class Geometry;

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

        // Transformation data:
        Scalar scale = 1; // Default to unity scale
        Vector3<Scalar> position = Vector3<Scalar>(0); // Default to origin
        Rotation<Scalar> rotation = Rotation<Scalar>(); // Default to Identity
        bool transform_changed = false;

        Scalar recip_scale = 1;
        Rotation<Scalar> inverse_rotation = Rotation<Scalar>(); // Default to Identity

        Geometry<Scalar>* parent;

        bool loaded;

        AABB<Scalar> bounds; // in body (geometry) space
        AABB<Scalar> bounds_world; // in world space

        BVHNode<Scalar>* bvhNode = nullptr;
        uint* triIdx = nullptr;

        BVH();

        BVH(AABB<Scalar> bounds);

        BVH(Triangle<Scalar>* triangles, uint num_triangles);

        ~BVH();

        void UpdateBounds(); // Update bounds based on the current transformations

        void UpdateNodeBounds( uint nodeIdx );

        void set_parent(Geometry<Scalar>* parent);

        void init(Triangle<Scalar>* triangles, uint num_triangles);
        void deinit();

        // Functions for Binned BVH building:
        Scalar FindBestSplitPlane( BVHNode<Scalar>& node, int& axis, Scalar& splitPos);
        Scalar CalculateNodeCost( BVHNode<Scalar>& node );
        void Subdivide( uint nodeIdx, int BINS);
        void Build(int BINS=8);

        // Function for ray traversal:
        void InnerIntersect( Ray<Scalar>& ray, const uint nodeIdx);
        void Intersect( Ray<Scalar>& ray, uint tile_number);

        void transform(Ray<Scalar> &ray);
        void transform(Vector3<Scalar> &vector);

        void inverse_transform(Ray<Scalar> &ray);
        void inverse_transform(Vector3<Scalar> &vector);

    private:
        mutex_t init_lock;
};

#endif