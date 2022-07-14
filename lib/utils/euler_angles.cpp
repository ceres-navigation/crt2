#include "utils/euler_angles.hpp"
#include "utils/rotation.hpp"

#include <math.h>
#include <iostream>

#define PI 3.14159265

// Basis rotations:
template<typename Scalar>
Rotation<Scalar> rotation_X(Scalar x){
    Scalar elements[3][3];
    elements[0][0] = 1;
    elements[0][1] = 0;
    elements[0][2] = 0;
    elements[1][0] = 0;
    elements[1][1] = cos ( x * PI / 180.0 );
    elements[1][2] = -sin ( x * PI / 180.0 );
    elements[2][0] = 0;
    elements[2][1] = sin ( x * PI / 180.0 );
    elements[2][2] = cos ( x * PI / 180.0 );
    return Rotation<Scalar>(elements);
};

template<typename Scalar>
Rotation<Scalar> rotation_Y(Scalar y){
    Scalar elements[3][3];
    elements[0][0] = cos ( y * PI / 180.0 );
    elements[0][1] = 0;
    elements[0][2] = sin ( y * PI / 180.0 );
    elements[1][0] = 0;
    elements[1][1] = 1;
    elements[1][2] = 0;
    elements[2][0] = -sin ( y * PI / 180.0 );
    elements[2][1] = 0;
    elements[2][2] = cos ( y * PI / 180.0 );
    return Rotation<Scalar>(elements);
};

template<typename Scalar>
Rotation<Scalar> rotation_Z(Scalar z){
    Scalar elements[3][3];
    elements[0][0] = cos ( z * PI / 180.0 );
    elements[0][1] = -sin ( z * PI / 180.0 );
    elements[0][2] = 0;
    elements[1][0] = sin ( z * PI / 180.0 );
    elements[1][1] = cos ( z * PI / 180.0 );
    elements[1][2] = 0;
    elements[2][0] = 0;
    elements[2][1] = 0;
    elements[2][2] = 1;
    return Rotation<Scalar>(elements);
};

template <typename Scalar>
Rotation<Scalar> XYZ_euler(Scalar x, Scalar y, Scalar z){
    return rotation_Z(z)*rotation_Y(y)*rotation_X(x);
};

template <typename Scalar>
Rotation<Scalar> XZY_euler(Scalar x, Scalar y, Scalar z){
    return rotation_Y(y)*rotation_Z(z)*rotation_X(x);
};

template <typename Scalar>
Rotation<Scalar> YXZ_euler(Scalar x, Scalar y, Scalar z){
    return rotation_Z(z)*rotation_X(x)*rotation_Y(y);
};

template <typename Scalar>
Rotation<Scalar> YZX_euler(Scalar x, Scalar y, Scalar z){
    return rotation_X(x)*rotation_Z(z)*rotation_Y(y);
};

template <typename Scalar>
Rotation<Scalar> ZXY_euler(Scalar x, Scalar y, Scalar z){
    return rotation_Y(y)*rotation_X(x)*rotation_Z(z);
};

template <typename Scalar>
Rotation<Scalar> ZYX_euler(Scalar x, Scalar y, Scalar z){
    return rotation_X(x)*rotation_Y(y)*rotation_Z(z);
};


// Explicitly Instantiate floats and doubles:
template Rotation<float> rotation_X<float>(float x);
template Rotation<float> rotation_Y<float>(float y);
template Rotation<float> rotation_Z<float>(float z);
template Rotation<float> XYZ_euler<float>(float x, float y, float z);
template Rotation<float> XZY_euler<float>(float x, float y, float z);
template Rotation<float> YXZ_euler<float>(float x, float y, float z);
template Rotation<float> YZX_euler<float>(float x, float y, float z);
template Rotation<float> ZXY_euler<float>(float x, float y, float z);
template Rotation<float> ZYX_euler<float>(float x, float y, float z);

template Rotation<double> rotation_X<double>(double x);
template Rotation<double> rotation_Y<double>(double y);
template Rotation<double> rotation_Z<double>(double z);
template Rotation<double> XYZ_euler<double>(double x, double y, double z);
template Rotation<double> XZY_euler<double>(double x, double y, double z);
template Rotation<double> YXZ_euler<double>(double x, double y, double z);
template Rotation<double> YZX_euler<double>(double x, double y, double z);
template Rotation<double> ZXY_euler<double>(double x, double y, double z);
template Rotation<double> ZYX_euler<double>(double x, double y, double z);