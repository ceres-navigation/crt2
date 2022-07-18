#include "scene.hpp"
#include "primitives/ray.hpp"
#include "lights/light.hpp"

#include "physics/spectral_radiance.hpp"

template <typename Scalar>
void backward_trace(Scene<Scalar>* scene, Ray<Scalar>& ray, std::vector<Light<Scalar>*> &lights, uint num_bounces, 
                    SpectralRadiance<Scalar> &pixel_radiance, uint tile_number);