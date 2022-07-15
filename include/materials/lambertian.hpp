#ifndef __LAMBERTIAN_H_
#define __LAMBERTIAN_H_
#include "materials/material.hpp"

#include "physics/spectral_radiance.hpp"
#include "utils/vector.hpp"
#include "lights/light.hpp"
#include "primitives/ray.hpp"

#include <random>

template <typename Scalar>
class Lambertian : public Material<Scalar> {
    private:

    public:
        // std::minstd_rand eng;
        // std::uniform_real_distribution<Scalar> d;
        std::default_random_engine generator;
        std::uniform_real_distribution<Scalar> distribution;

        Lambertian();

        ~Lambertian();

        SpectralRadiance<Scalar> get_color(Ray<Scalar> &light_ray, Ray<Scalar> &view_ray,
                                           Vector3<Scalar> &normal, Vector2<Scalar> &interp_uv);

        Vector3<Scalar> bounce_ray(Ray<Scalar> &ray, Vector3<Scalar> &normal, Vector2<Scalar> &interp_uv);
};

#endif