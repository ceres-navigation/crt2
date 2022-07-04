#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"
#include "vector_math/vector.hpp"

template <typename Scalar>
void intersect_triangle(Ray<Scalar>& ray, const Triangle<Scalar>& triangle){
    const Vector3<Scalar> edge1 = triangle.vertex1 - triangle.vertex0;
    const Vector3<Scalar> edge2 = triangle.vertex2 - triangle.vertex0;
    const Vector3<Scalar> h = cross<Scalar>(ray.direction, edge2);
    const Scalar a = dot<Scalar>(edge1, h);

    const Scalar f = 1/a;
    const Vector3<Scalar> s = ray.origin - triangle.vertex0;
    const Scalar u = f * dot<Scalar>(s,h);
    if (u < 0 || u > 1) return;
    const Vector3<Scalar> q = cross<Scalar>(s, edge1);
    const Scalar v = f* dot<Scalar>(ray.direction, q);
    if (v < 0 || u + v > 1) return;
    const Scalar t = f * dot<Scalar>(edge2, q);
    if (t > std::numeric_limits<Scalar>::min() && t < ray.hit.t){
        ray.hit.t = t;
        ray.hit.u = u;
        ray.hit.v = v;
        ray.hit.n = cross<Scalar>(edge2, edge1);
    }
};

template <typename Scalar>
Scalar intersect_AABB(const Ray<Scalar> ray, const Vector3<Scalar> bmin, const Vector3<Scalar> bmax){
    Scalar tx1 = (bmin[0] - ray.origin[0]) / ray.direction[0], tx2 = (bmax[0] - ray.origin[0]) / ray.direction[0];
    Scalar tmin = std::min( tx1, tx2 ), tmax = std::max( tx1, tx2 );
    Scalar ty1 = (bmin[1] - ray.origin[1]) / ray.direction[1], ty2 = (bmax[1] - ray.origin[1]) / ray.direction[1];
    tmin = std::max( tmin, std::min( ty1, ty2 ) ), tmax = std::min( tmax, std::max( ty1, ty2 ) );
    Scalar tz1 = (bmin[2] - ray.origin[2]) / ray.direction[2], tz2 = (bmax[2] - ray.origin[2]) / ray.direction[2];
    tmin = std::max( tmin, std::min( tz1, tz2 ) ), tmax = std::min( tmax, std::max( tz1, tz2 ) );
    if (tmax >= tmin && tmin < ray.hit.t && tmax > 0) return tmin; else return std::numeric_limits<Scalar>::max();
};


// Explicitly Instantiate floats and doubles:
template struct Triangle<float>;
template struct Triangle<double>;

template struct TriangleData<float>;
template struct TriangleData<double>;

template void intersect_triangle<float>(Ray<float>& ray, const Triangle<float>& triangle);
template void intersect_triangle<double>(Ray<double>& ray, const Triangle<double>& triangle);

template float intersect_AABB<float>(const Ray<float> ray, const Vector3<float> bmin, const Vector3<float> bmax);
template double intersect_AABB<double>(const Ray<double> ray, const Vector3<double> bmin, const Vector3<double> bmax);