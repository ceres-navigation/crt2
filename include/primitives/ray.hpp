#ifndef __RAY_H_
#define __RAY_H_

#include "utils/vector.hpp"
// #include "primitives/geometry.hpp"

// Forward declaration of Geometry class:
template <typename Scalar>
class Geometry; 


template <typename Scalar>
struct Intersection {
    Scalar t = std::numeric_limits<Scalar>::max();
    Scalar u;
    Scalar v;
    Geometry<Scalar>* geometry = nullptr;
    uint triIdx;
};


template <typename Scalar>
struct Ray {
    Vector3<Scalar> origin;
    Vector3<Scalar> direction;
    Vector3<Scalar> recip_direction;
    Scalar t;
    Intersection<Scalar> hit;

    Ray();

    Ray(Vector3<Scalar> origin, Vector3<Scalar> direction);

    Ray(Vector3<Scalar> origin, Vector3<Scalar> direction, Scalar t_max);
};

#endif