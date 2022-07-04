#include "lights/point_light.hpp"
#include "primitives/ray.hpp"

template <typename Scalar>
PointLight<Scalar>::PointLight(Scalar intensity){
    this -> intensity = intensity;
};

template <typename Scalar>
PointLight<Scalar>::PointLight(const PointLight<Scalar> &original){
    this -> intensity = original.intensity;
    this -> position  = original.position;
    this -> rotation  = original.rotation;
}

template <typename Scalar>
Ray<Scalar> PointLight<Scalar>::sample_ray(Vector3<Scalar> origin){
    Vector3<Scalar> light_direction = normalize(this->position - origin);
    return Ray<Scalar>(origin, light_direction, length(this->position - origin));
};

template <typename Scalar>
Scalar PointLight<Scalar>::get_intensity(Vector3<Scalar> point) { 
    return std::min(this->intensity / dot(point - this->position, point - this->position), Scalar(10000));
};


// Explicitly Instantiate floats and doubles:
template class PointLight<float>;
template class PointLight<double>;