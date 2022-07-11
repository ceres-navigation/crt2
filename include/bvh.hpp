#ifndef __BVH_H_
#define __BVH_H_

#include <iostream>
#include <stdint.h>

#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"
#include "primitives/aabb.hpp"
#include "vector_math/vector.hpp"
#include "vector_math/rotation.hpp"

// TRANSFORM NODE STUFF
// MODE THIS
template <typename Scalar>
class TransformNode {
    public:
        Vector3<Scalar> translation;

        TransformNode(Vector3<Scalar>);

        void apply(Ray<Scalar>& ray);
        void invert(Ray<Scalar>& ray);

        void apply(Vector3<Scalar>& ray);
        void invert(Vector3<Scalar>& ray);
};

template <typename Scalar>
TransformNode<Scalar>::TransformNode(Vector3<Scalar> translation){
    this->translation = translation;
};

template <typename Scalar>
void TransformNode<Scalar>::apply(Ray<Scalar>& ray){
    ray.origin = ray.origin + this->translation;
};

template <typename Scalar>
void TransformNode<Scalar>::invert(Ray<Scalar>& ray){
    ray.origin = ray.origin - this->translation;
}

template <typename Scalar>
void TransformNode<Scalar>::apply(Vector3<Scalar>& vec){
    vec = vec + this->translation;
};

template <typename Scalar>
void TransformNode<Scalar>::invert(Vector3<Scalar>& vec){
    vec = vec - this->translation;
}
// ========================



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

        // Transformation Nodes:
        int num_transformations = 0;
        TransformNode<Scalar>* transform_nodes = nullptr;

        AABB<Scalar> bounds; // in world space

        BVHNode<Scalar>* bvhNode = nullptr;
        uint* triIdx = nullptr;

        BVH();

        BVH(Triangle<Scalar>* triangles, uint num_triangles);

        ~BVH();

        void UpdateBounds(); // Update bounds based on the current transformations

        void UpdateNodeBounds( uint nodeIdx );

        // Functions for Binned BVH building:
        Scalar FindBestSplitPlane( BVHNode<Scalar>& node, int& axis, Scalar& splitPos);
        Scalar CalculateNodeCost( BVHNode<Scalar>& node );
        void Subdivide( uint nodeIdx, int BINS);
        void Build(int BINS=8);

        AABB<Scalar> Bounds();

        // Function for ray traversal:
        void InnerIntersect( Ray<Scalar>& ray, const uint nodeIdx);
        void Intersect( Ray<Scalar>& ray, const uint nodeIdx = 0);

        void UpdateTransforms();
};

#endif