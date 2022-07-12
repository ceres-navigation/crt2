#ifndef __LIGHT_H_
#define __LIGHT_H_

#include "utils/rigid_body.hpp"
#include "utils/vector.hpp"
#include "primitives/ray.hpp"

// Abstract light class:
template <typename Scalar>
class Light: public RigidBody<Scalar> {
    public:
        Scalar intensity;

        // Abstract methods:
        virtual Ray<Scalar> sample_ray(Vector3<Scalar> origin) = 0;
        virtual Scalar get_intensity(Vector3<Scalar> point) = 0;
};

#endif