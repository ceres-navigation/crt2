#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"
#include "utils/vector.hpp"

template <typename Scalar>
void intersect_triangle(Ray<Scalar>& ray, const Triangle<Scalar>& triangle){
    const Vector3<Scalar> edge1 = triangle.vertex1 - triangle.vertex0;
    const Vector3<Scalar> edge2 = triangle.vertex2 - triangle.vertex0;
    const Vector3<Scalar> h = cross<Scalar>(ray.direction, edge2);
    const Scalar a = dot<Scalar>(edge1, h);
    if (a > -std::numeric_limits<Scalar>::min() && a < std::numeric_limits<Scalar>::min()){
        return;
    }

    const Scalar f = 1/a;
    const Vector3<Scalar> s = ray.origin - triangle.vertex0;
    const Scalar u = f * dot<Scalar>(s,h);
    if (u < 0 || u > 1){
        return;
    }

    const Vector3<Scalar> q = cross<Scalar>(s, edge1);
    const Scalar v = f* dot<Scalar>(ray.direction, q);
    if (v < 0 || u + v > 1){
        return;
    }

    const Scalar t = f * dot<Scalar>(edge2, q);
    if (t > std::numeric_limits<Scalar>::min() && t < ray.hit.t){
        ray.hit.t = t;
        ray.hit.u = u;
        ray.hit.v = v;
    }
};

// Explicitly Instantiate floats and doubles:
template struct Triangle<float>;
template struct Triangle<double>;

template struct TriangleData<float>;
template struct TriangleData<double>;

template void intersect_triangle<float>(Ray<float>& ray, const Triangle<float>& triangle);
template void intersect_triangle<double>(Ray<double>& ray, const Triangle<double>& triangle);