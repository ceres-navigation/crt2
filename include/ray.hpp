#ifndef __RAY_H_
#define __RAY_H_

#include "vector_math/vector.hpp"

template <typename Scalar>
struct Intersection {
    Vector3<Scalar> n;
    float t = std::numeric_limits<Scalar>::max();
    float u, v;
};

template <typename Scalar>
struct Ray {
    Vector3<Scalar> origin;
    Vector3<Scalar> direction;
    Scalar t;
    Intersection<Scalar> hit;

    Ray();

    Ray(Vector3<Scalar> origin, Vector3<Scalar> direction);

    Ray(Vector3<Scalar> origin, Vector3<Scalar> direction, Scalar t_max);
};

#endif