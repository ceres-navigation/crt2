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
    Vector3<Scalar> vn0;
    Vector3<Scalar> vn1;
    Vector3<Scalar> vn2;

    Vector3<Scalar> face_normal;

    Vector2<Scalar> uv0;
    Vector2<Scalar> uv1;
    Vector2<Scalar> uv2;
};

template <typename Scalar>
void intersect_triangle(Ray<Scalar>& ray, const Triangle<Scalar>& triangle);

#endif