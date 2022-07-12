#ifndef __AABB_H_
#define __AABB_H_

#include "utils/vector.hpp"
#include "primitives/ray.hpp"

template <typename Scalar>
struct AABB { 
    Vector3<Scalar> bmin = Vector3<Scalar>(std::numeric_limits<Scalar>::max());
    Vector3<Scalar> bmax = Vector3<Scalar>(-std::numeric_limits<Scalar>::max());

    void grow(Vector3<Scalar> p);

    void grow(AABB<Scalar>& b);

    Scalar area();
};

template<typename Scalar>
Scalar intersect_aabb(const Ray<Scalar>& ray, const Vector3<Scalar> bmin, const Vector3<Scalar> bmax);

#endif