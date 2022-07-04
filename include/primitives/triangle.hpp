#ifndef __TRIANGLE_H_
#define __TRIANGLE_H_

#include "vector_math/vector.hpp"
#include "ray.hpp"

template <typename Scalar>
struct Triangle {
    Vector3<Scalar> vertex0;
    Vector3<Scalar> vertex1;
    Vector3<Scalar> vertex2;
    Vector3<Scalar> centroid;
};

template <typename Scalar>
void intersect_triangle(Ray<Scalar>& ray, const Triangle<Scalar>& triangle);

template <typename Scalar>
Scalar intersect_AABB(const Ray<Scalar> ray, const Vector3<Scalar> bmin, const Vector3<Scalar> bmax);

#endif