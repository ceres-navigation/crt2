#ifndef __POINT_LIGHT_H_
#define __POINT_LIGHT_H_

#include "lights/light.hpp"
#include "utils/vector.hpp"
#include "primitives/ray.hpp"

template <typename Scalar>
class PointLight: public Light<Scalar>  {
    public:
        PointLight(Scalar intensity);

        // Copy constructor:
        PointLight(const PointLight<Scalar> &original);

        Ray<Scalar> sample_ray(Vector3<Scalar> origin);

        Scalar get_intensity(Vector3<Scalar> point);
};

#endif