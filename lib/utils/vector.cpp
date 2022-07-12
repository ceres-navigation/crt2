#include "utils/vector.hpp"
#include <array>
#include <cmath>


// TODO: This was borrowed from another library...  implement my own at somepoint!



// Explicitly Instantiate floats and doubles:
template struct Vec<float,3>;
template struct Vec<double,3>;

template struct Vec<float,2>;
template struct Vec<double,2>;

template float dot<float,3>(const Vector3<float> &a, const Vector3<float> &b);
template double dot<double,3>(const Vector3<double> &a, const Vector3<double> &b);

template float length<float,3>(const Vector3<float> &a);
template double length<double,3>(const Vector3<double> &a);

template Vector3<float> normalize<float,3>(const Vector3<float> &a);
template Vector3<double> normalize<double,3>(const Vector3<double> &a);

template Vector3<float> cross<float>(const Vector3<float> &a, const Vector3<float> &b);
template Vector3<double> cross<double>(const Vector3<double> &a, const Vector3<double> &b);