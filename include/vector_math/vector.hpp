#ifndef __VECTOR_H_
#define __VECTOR_H_

#include <cmath>

template <typename Scalar>
struct Vector3{
    Scalar elements[3];

    Vector3();

    Vector3(Scalar value);

    Vector3(Scalar x, Scalar y, Scalar z);

    Vector3 operator + (Vector3<Scalar> vec2){
        Scalar a = this->elements[0] + vec2[0];
        Scalar b = this->elements[1] + vec2[1];
        Scalar c = this->elements[2] + vec2[2];
        return Vector3<Scalar>(a,b,c);
    }

    Vector3 operator - (Vector3<Scalar> vec2){
        Scalar a = this->elements[0] - vec2[0];
        Scalar b = this->elements[1] - vec2[1];
        Scalar c = this->elements[2] - vec2[2];
        return Vector3<Scalar>(a,b,c);
    }

    Vector3 operator * (Scalar s){
        Scalar a = this->elements[0]*s;
        Scalar b = this->elements[1]*s;
        Scalar c = this->elements[2]*s;
        return Vector3<Scalar>(a,b,c);
    }

    Scalar& operator [] (size_t i) { return elements[i]; };
    Scalar  operator [] (size_t i) const { return elements[i]; };
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