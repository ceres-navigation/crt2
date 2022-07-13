#include "scene.hpp"
#include "primitives/ray.hpp"
#include "lights/light.hpp"

template <typename Scalar>
inline void backward_trace(Scene<Scalar>* scene, Ray<Scalar>& ray, std::vector<Light<Scalar>*> &lights, Vector3<Scalar> &pixel_radiance);