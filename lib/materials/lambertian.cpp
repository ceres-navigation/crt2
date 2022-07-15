#include "materials/lambertian.hpp"
#include "materials/sampling.hpp"

#include "utils/vector.hpp"
#include "physics/spectral_radiance.hpp"
#include "primitives/ray.hpp"

#include "lights/light.hpp"

#include <random>

template<typename Scalar>
Lambertian<Scalar>::Lambertian(){
    distribution = std::uniform_real_distribution<Scalar>(0.0,1.0);
};

template<typename Scalar>
Lambertian<Scalar>::~Lambertian(){
};

template <typename Scalar>
SpectralRadiance<Scalar> Lambertian<Scalar>::get_color(Ray<Scalar> &light_ray, Ray<Scalar> &view_ray, 
                                                       Vector3<Scalar> &normal, Vector2<Scalar> &interp_uv){
    Scalar L_dot_N = dot(light_ray.direction, normal);

    return SpectralRadiance<Scalar>(L_dot_N);
};

template <typename Scalar>
Vector3<Scalar> Lambertian<Scalar>::bounce_ray(Ray<Scalar> &ray, Vector3<Scalar> &normal, Vector2<Scalar> &interp_uv){
    Scalar r1 = distribution(generator);
    Scalar r2 = distribution(generator);
    return cosine_importance<Scalar>(normal,r1,r2);
};

// Explicitly Instantiate floats and doubles:
template class Lambertian<float>;
template class Lambertian<double>;