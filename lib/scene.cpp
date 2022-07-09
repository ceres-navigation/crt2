#include "scene.hpp"
#include "geometry.hpp"

#include "acceleration/bvh.hpp"

#include "primitives/ray.hpp"

template <typename Scalar>
Scene<Scalar>::Scene(Geometry<Scalar>* geometries, uint num_geometries){
    this->geometries = geometries;
    this->num_geometries = num_geometries;
};

template <typename Scalar>
Scene<Scalar>::~Scene(){
};

template <typename Scalar>
void Scene<Scalar>::build_bvh(int BINS){
    for (uint i = 0; i < this->num_geometries; i++){
        this->geometries[i].build_bvh(BINS);
    }
};

template <typename Scalar>
void Scene<Scalar>::fast_build_bvh(){
    for (uint i = 0; i < this->num_geometries; i++){
        this->geometries[i].fast_build_bvh();
    }
};

template <typename Scalar>
void Scene<Scalar>::intersect(Ray<Scalar>& ray){
    for (uint i = 0; i < this->num_geometries; i++){
        this->geometries[i].intersect(ray);
    }
};

// Explicitly Instantiate floats and doubles:
template class Scene<float>;
template class Scene<double>;