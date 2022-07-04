#include "vector_math/vector.hpp"
#include <cmath>

template <typename Scalar>
Vector3<Scalar>::Vector3(){
    this->elements[0] = 0;
    this->elements[1] = 0;
    this->elements[2] = 0;
};

template <typename Scalar>
Vector3<Scalar>::Vector3(Scalar value){
    this->elements[0] = value;
    this->elements[1] = value;
    this->elements[2] = value;
};

template <typename Scalar>
Vector3<Scalar>::Vector3(Scalar x, Scalar y, Scalar z){
    this->elements[0] = x;
    this->elements[1] = y;
    this->elements[2] = z;
};

template <typename Scalar>
Vector3<Scalar> vector_scale(Vector3<Scalar> vector, Scalar scale){
    vector[0] = scale*vector[0];
    vector[1] = scale*vector[1];
    vector[2] = scale*vector[2];
    return vector;
};

template <typename Scalar>
Vector3<Scalar> vector_add(Vector3<Scalar> vector, Vector3<Scalar> vector2){
    return Vector3<Scalar>(vector[0] + vector2[0], vector[1] + vector2[1], vector[2] + vector2[2]);
};

template <typename Scalar>
Vector3<Scalar> cross(Vector3<Scalar> vector, Vector3<Scalar> vector2){
    Scalar a = vector[1]*vector2[2] - vector[2]*vector2[1];
    Scalar b = vector[2]*vector2[0] - vector[0]*vector2[2];
    Scalar c = vector[0]*vector2[1] - vector[1]*vector2[0];
    return Vector3<Scalar>(a,b,c);
};

template <typename Scalar>
Scalar dot(Vector3<Scalar> vector, Vector3<Scalar> vector2){
    Scalar sum = vector[0] * vector2[0];
    for (size_t i = 1; i < 3; ++i)
        sum += vector[i] * vector2[i];
    return sum;
};

template <typename Scalar>
Scalar length(Vector3<Scalar> vector){
    return std::sqrt(dot(vector, vector));
};

template <typename Scalar>
Vector3<Scalar> normalize(Vector3<Scalar> vector){
    auto inv = Scalar(1) / length(vector);
    return vector * inv;
};

// Explicitly Instantiate floats and doubles:
template struct Vector3<float>;
template struct Vector3<double>;

template Vector3<float> vector_scale<float>(Vector3<float> vector, float scale);
template Vector3<double> vector_scale<double>(Vector3<double> vector, double scale);

template Vector3<float> vector_add<float>(Vector3<float> vector, Vector3<float> vector2);
template Vector3<double> vector_add<double>(Vector3<double> vector, Vector3<double> vector2);

template Vector3<float> cross<float>(Vector3<float> vector, Vector3<float> vector2);
template Vector3<double> cross<double>(Vector3<double> vector, Vector3<double> vector2);

template float dot<float>(Vector3<float> vector, Vector3<float> vector2);
template double dot<double>(Vector3<double> vector, Vector3<double> vector2);

template float length<float>(Vector3<float> vector);
template double length<double>(Vector3<double> vector);

template Vector3<float> normalize<float>(Vector3<float> vector);
template Vector3<double> normalize<double>(Vector3<double> vector);