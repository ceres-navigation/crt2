#ifndef __TRIANGLE_H_
#define __TRIANGLE_H_

#include "utils/vector.hpp"
#include "primitives/ray.hpp"

template <typename Scalar>
struct Triangle {
    Vector3<Scalar> vertex0;
    Vector3<Scalar> vertex1;
    Vector3<Scalar> vertex2;

    Vector3<Scalar> centroid;
};

template <typename Scalar>
struct TriangleData {
    Vector3<Scalar> normal0;
    Vector3<Scalar> normal1;
    Vector3<Scalar> normal2;

    Vector2<Scalar> u;
    Vector2<Scalar> v;
};

template <typename Scalar>
void intersect_triangle(Ray<Scalar>& ray, const Triangle<Scalar>& triangle);

#endif