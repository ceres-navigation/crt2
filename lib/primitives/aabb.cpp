#include "primitives/aabb.hpp"
#include "vector_math/vector.hpp"

template <typename Scalar>
void AABB<Scalar>::grow(Vector3<Scalar> p){
    bmin = min(bmin, p);
    bmax = max(bmax, p);
};

template <typename Scalar>
void AABB<Scalar>::grow(AABB<Scalar>& b) {
    if (b.bmin[0] != std::numeric_limits<Scalar>::max()) {
        grow( b.bmin );
        grow( b.bmax );
    }
};

template <typename Scalar>
Scalar intersect_aabb( const Ray<Scalar>& ray, const Vector3<Scalar> bmin, const Vector3<Scalar> bmax ) {
    Scalar tx1 = (bmin[0] - ray.origin[0]) / ray.direction[0], tx2 = (bmax[0] - ray.origin[0]) / ray.direction[0];
    Scalar tmin = std::min( tx1, tx2 ), tmax = std::max( tx1, tx2 );
    Scalar ty1 = (bmin[1] - ray.origin[1]) / ray.direction[1], ty2 = (bmax[1] - ray.origin[1]) / ray.direction[1];
    tmin = std::max( tmin, std::min( ty1, ty2 ) ), tmax = std::min( tmax, std::max( ty1, ty2 ) );
    Scalar tz1 = (bmin[2] - ray.origin[2]) / ray.direction[2], tz2 = (bmax[2] - ray.origin[2]) / ray.direction[2];
    tmin = std::max( tmin, std::min( tz1, tz2 ) ), tmax = std::min( tmax, std::max( tz1, tz2 ) );
    if (tmax >= tmin && tmin < ray.hit.t && tmax > 0) {
        return tmin;
    } 
    else {
        return std::numeric_limits<Scalar>::max();
    }
};

template float intersect_aabb<float>(const Ray<float> &ray, const Vector3<float> bmin, const Vector3<float> bmax);
template double intersect_aabb<double>(const Ray<double> &ray, const Vector3<double> bmin, const Vector3<double> bmax);