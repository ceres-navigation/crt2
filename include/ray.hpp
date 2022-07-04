#ifndef __RAY_H_
#define __RAY_H_

#include "vector_math/vector.hpp"

template <typename Scalar>
struct Ray {
    Vector3<Scalar> origin;
    Vector3<Scalar> direction;
    Scalar t;

    Ray();

    Ray(Vector3<Scalar> origin, Vector3<Scalar> direction);

    Ray(Vector3<Scalar> origin, Vector3<Scalar> direction, Scalar t_max);
};

#endif