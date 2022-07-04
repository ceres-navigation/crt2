#include "sensors/simple_sensor.hpp"

template <typename Scalar>
SimpleSensor<Scalar>::SimpleSensor(Scalar resolution[2], Scalar size[2]){
    this -> resolution[0] = resolution[0];
    this -> resolution[1] = resolution[1];
    this -> size[0] = size[0];
    this -> size[1] = size[1];

    this -> center[0] = resolution[0]/2.0;
    this -> center[1] = resolution[1]/2.0;
};

template <typename Scalar>
SimpleSensor<Scalar>::SimpleSensor(Scalar resolution[2], Scalar size[2], Scalar center[2]){
    this -> resolution[0] = resolution[0];
    this -> resolution[1] = resolution[1];
    this -> size[0] = size[0];
    this -> size[1] = size[1];
    
    this -> center[0] = center[0];
    this -> center[1] = center[1];
};

template <typename Scalar>
Scalar SimpleSensor<Scalar>::get_resolution_h(){
    return this->resolution[0];
};

template <typename Scalar>
Scalar SimpleSensor<Scalar>::get_resolution_v(){
    return this->resolution[1];
};


// Explicitly Instantiate floats and doubles:
template class SimpleSensor<float>;
template class SimpleSensor<double>;