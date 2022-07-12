#ifndef __CAMERA_H_
#define __CAMERA_H_

#include "utils/rigid_body.hpp"
#include "sensors/sensor.hpp"
#include "primitives/ray.hpp"

template <typename Scalar>
class Camera: public RigidBody<Scalar> {
    public:
        Scalar scale[2];
        Scalar center[2];
        Sensor<Scalar> *sensor;

        bool z_positive;
        Scalar focal_length;

        Camera() {};

        ~Camera() {};

        virtual Ray<Scalar> pixel_to_ray(Scalar u, Scalar v) = 0;
};

#endif