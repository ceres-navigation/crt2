#ifndef __VECTOR_H_
#define __VECTOR_H_

#include <cmath>

template <typename Scalar>
struct Vector3{
    Scalar elements[3];

    Vector3();

    Vector3(Scalar value);

    Vector3(Scalar x, Scalar y, Scalar z);

    Scalar& operator [] (size_t i) { return elements[i]; };
    Scalar  operator [] (size_t i) const { return elements[i]; };
};

template <typename Scalar>
Vector3<Scalar> operator + (const Vector3<Scalar> vec1, Vector3<Scalar> vec2){
    Scalar a = vec1[0] + vec2[0];
    Scalar b = vec1[1] + vec2[1];
    Scalar c = vec1[2] + vec2[2];
    return Vector3<Scalar>(a,b,c);
};

template <typename Scalar>
Vector3<Scalar> operator - (const Vector3<Scalar> vec1, Vector3<Scalar> vec2){
    Scalar a = vec1[0] - vec2[0];
    Scalar b = vec1[1] - vec2[1];
    Scalar c = vec1[2] - vec2[2];
    return Vector3<Scalar>(a,b,c);
};

template <typename Scalar>
Vector3<Scalar> operator * (const Vector3<Scalar> vec1, const Scalar s){
    Scalar a = vec1[0]*s;
    Scalar b = vec1[1]*s;
    Scalar c = vec1[2]*s;
    return Vector3<Scalar>(a,b,c);
};

template <typename Scalar>
Vector3<Scalar> vector_scale(Vector3<Scalar> vector, Scalar scale);

template <typename Scalar>
Vector3<Scalar> vector_add(Vector3<Scalar> vector, Vector3<Scalar> vector2);

template <typename Scalar>
Vector3<Scalar> cross(Vector3<Scalar> vector, Vector3<Scalar> vector2);

template <typename Scalar>
Scalar dot(Vector3<Scalar> vector, Vector3<Scalar> vector2);

template <typename Scalar>
Scalar length(Vector3<Scalar> vector);

template <typename Scalar>
Vector3<Scalar> normalize(Vector3<Scalar> vector);


#endif