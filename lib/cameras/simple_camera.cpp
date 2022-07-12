#include "sensors/sensor.hpp"

#include "cameras/simple_camera.hpp"
#include "utils/vector.hpp"

#include "primitives/ray.hpp"


template <typename Scalar>
SimpleCamera<Scalar>::SimpleCamera(Scalar focal_length, Sensor<Scalar> &sensor, bool z_positive){
    this -> focal_length = focal_length;
    this -> z_positive   = z_positive;
    this -> sensor = &sensor;

    // Extract required information:
    this -> center[0] = sensor.center[0];
    this -> center[1] = sensor.center[1];

    // Calcualte scale between sensor and camera:
    this -> scale[0] = sensor.resolution[0]/sensor.size[0];
    this -> scale[1] = sensor.resolution[1]/sensor.size[1];
};

template <typename Scalar>
SimpleCamera<Scalar>::SimpleCamera(const SimpleCamera<Scalar> &original) {
    this -> focal_length = original.focal_length;
    this -> z_positive = original.z_positive;
    this -> sensor = original.sensor;

    this -> center[0] = original.center[0];
    this -> center[1] = original.center[1];

    this -> scale[0]  = original.scale[0];
    this -> scale[1]  = original.scale[1];

    //TODO: move to RigidBody copy:
    this -> position = original.position;
    this -> rotation = original.rotation;
};

template <typename Scalar>
Ray<Scalar> SimpleCamera<Scalar>::pixel_to_ray(Scalar u, Scalar v) {
    // Generate rays in the camera frame:
    Vector3<Scalar> dir;
    if (this->z_positive){
        dir = normalize(Vector3<Scalar>((-this->center[0]+u+0.5)/this->scale[0], (this->center[1]-v-0.5)/this->scale[1], this->focal_length));
    }
    else {
        dir = normalize(Vector3<Scalar>((-this->center[0]+u+0.5)/this->scale[0], (this->center[1]-v-0.5)/this->scale[1], -this->focal_length));
    } 

    // Rotate rays to the world frame (NOTE: the TRANSPOSE of the provided rotation is used for this)
    dir = this->rotation.rotate(dir);

    // Return the ray object:
    Ray<Scalar> ray(this->position, dir);
    return ray;
};


// Explicitly Instantiate floats and doubles:
template class SimpleCamera<float>;
template class SimpleCamera<double>;