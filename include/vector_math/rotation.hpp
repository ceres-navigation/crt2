#ifndef __MATRIX_H_
#define __MATRIX_H_

#include "vector_math/vector.hpp"

template <typename Scalar>
struct Rotation{
    Scalar elements[3][3];

    Rotation(Scalar elements[3][3]);

    Rotation(){
        this -> elements[0][0] = 1;
        this -> elements[0][1] = 0;
        this -> elements[0][2] = 0;
        this -> elements[1][0] = 0;
        this -> elements[1][1] = 1;
        this -> elements[1][2] = 0;
        this -> elements[2][0] = 0;
        this -> elements[2][1] = 0;
        this -> elements[2][2] = 1;
    }

    Vector3<Scalar> rotate(Vector3<Scalar>);

    Rotation<Scalar> transpose();
};


#endif