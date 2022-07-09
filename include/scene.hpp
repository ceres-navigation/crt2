#ifndef __SCENE_H_
#define __SCENE_H_

#include "geometry.hpp"

template <typename Scalar>
class Scene{
    public:
        Geometry<Scalar>* geometries;
        uint num_geometries;
        // TLAS<Scalar> tlas;

        Scene(Geometry<Scalar>* geometries, uint num_geometries);

        ~Scene();

        void build_bvh(int BINS=8);
        void fast_build_bvh();

        void intersect( Ray<Scalar>& ray);
};

#endif