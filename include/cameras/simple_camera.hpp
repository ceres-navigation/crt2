#ifndef __PINHOLE_CAMERA_H_
#define __PINHOLE_CAMERA_H_

#include "cameras/camera.hpp"
#include "sensors/sensor.hpp"
#include "primitives/ray.hpp"

template <typename Scalar>
class SimpleCamera: public Camera<Scalar> {
    public:
        SimpleCamera(Scalar focal_length, Sensor<Scalar> &sensor, bool z_positive);

        SimpleCamera(const SimpleCamera<Scalar> &original);

        Ray<Scalar> pixel_to_ray(Scalar u, Scalar v);
};

#endif