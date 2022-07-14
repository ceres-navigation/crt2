#ifndef __MATERIAL_H_
#define __MATERIAL_H_

#include "physics/spectral_radiance.hpp"
#include "utils/vector.hpp"
#include "lights/light.hpp"
#include "primitives/ray.hpp"

#include <random>

template <typename Scalar>
class Material {
    public:
        Material() {};

        ~Material() {};

        virtual SpectralRadiance<Scalar> illumination(Light<Scalar>* light, Ray<Scalar> &light_ray, Ray<Scalar> &view_ray,
                                                      Vector3<Scalar> &intersect_point, Vector3<Scalar> &normal, Vector2<Scalar> &interp_uv) = 0;

        virtual Vector3<Scalar> bounce_ray(Vector3<Scalar> &normal, Vector2<Scalar> &interp_uv) = 0;
};

#endif 