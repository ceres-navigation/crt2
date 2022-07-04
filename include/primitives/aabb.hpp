#ifndef __AABB_H_
#define __AABB_H_

#include "vector_math/vector.hpp"

template <typename Scalar>
struct AABB 
{ 
    Vector3<Scalar> bmin = Vector3<Scalar>(std::numeric_limits<Scalar>::max());
    Vector3<Scalar> bmax = Vector3<Scalar>(std::numeric_limits<Scalar>::min());

    void grow(Vector3<Scalar> p);

    void grow(AABB<Scalar>& b);

    float area() 
    { 
        Vector3<Scalar> e = bmax - bmin; // box extent
        return e[0] * e[1] + e[1] * e[2] + e[2] * e[0]; 
    }
};

#endif