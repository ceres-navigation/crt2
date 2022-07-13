#include "scene.hpp"
#include "primitives/ray.hpp"
#include "lights/light.hpp"

template <typename Scalar>
void backward_trace(Scene<Scalar>* scene, Ray<Scalar>& ray, std::vector<Light<Scalar>*> &lights, uint num_bounces, Vector3<Scalar> &pixel_radiance);