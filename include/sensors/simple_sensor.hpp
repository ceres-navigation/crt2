#ifndef __SIMPLE_SENSOR_H_
#define __SIMPLE_SENSOR_H_

#include "sensors/sensor.hpp"

template <typename Scalar>
class SimpleSensor: public Sensor<Scalar>{
    public:
        SimpleSensor(Scalar resolution[2], Scalar size[2]);

        SimpleSensor(Scalar resolution[2], Scalar size[2], Scalar center[2]);

        Scalar get_resolution_h();

        Scalar get_resolution_v();
};

#endif