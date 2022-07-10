#ifndef __SCENE_H_
#define __SCENE_H_

#include "geometry.hpp"
#include "graph_nodes/tlas_node.hpp"

template <typename Scalar>
class Scene{
    public:
        Geometry<Scalar>* geometries;
        uint num_geometries;

        Scene(Geometry<Scalar>* geometries, uint num_geometries);

        ~Scene();

        void build_bvh(int BINS=8);

        void intersect( Ray<Scalar>& ray);
};

#endif