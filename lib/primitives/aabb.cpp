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