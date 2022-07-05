#ifndef __BVH_H_
#define __BVH_H_

#include <iostream>
#include <stdint.h>

#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"
#include "primitives/aabb.hpp"
#include "vector_math/vector.hpp"

#define BINS 8

template <typename Scalar>
struct BVHNode{
    Vector3<Scalar> aabb_min;
    Vector3<Scalar> aabb_max;

    uint32_t left_first;
    uint32_t num_triangles; 

    bool is_leaf() {return num_triangles > 0;}
};

template <typename Scalar>
class BVH{
    public:
        BVH();

        ~BVH();

        AABB<Scalar> bounds();

        void build(Triangle<Scalar> *triangles, uint32_t num_triangles);

        void intersect(Ray<Scalar> &ray, Triangle<Scalar> *triangles);

    private:
        BVHNode<Scalar> *bvh_node = 0;
        uint32_t nodes_used = 0;
        uint32_t *triangle_index = 0;

        void update_node_bounds(uint node_index, Triangle<Scalar> *triangles);

        void subdivide(uint node_index, Triangle<Scalar> *triangles);

        Scalar find_best_split_plane(BVHNode<Scalar> &node, int &axis, Scalar &split_position);
};

#endif