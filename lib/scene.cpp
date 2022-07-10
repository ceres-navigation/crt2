#include "scene.hpp"
#include "geometry.hpp"

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
};

template <typename Scalar>
void Scene<Scalar>::intersect(Ray<Scalar>& ray){
};

// Explicitly Instantiate floats and doubles:
template class Scene<float>;
template class Scene<double>;