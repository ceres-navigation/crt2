#ifndef __SENSOR_H_
#define __SENSOR_H_

#include <stdint.h>

template <typename Scalar>
class Sensor {
    public:
        Scalar center[2];
        Scalar resolution[2];
        Scalar size[2];

        Sensor() {};

        ~Sensor() {};

        virtual Scalar get_resolution_h() = 0;

        virtual Scalar get_resolution_v() = 0;
};

#endif