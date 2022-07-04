#include "vector_math/rotation.hpp"

template <typename Scalar>
Rotation<Scalar>::Rotation(Scalar elements[3][3]){
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            this -> elements[i][j] = elements[i][j];
        }
    }
}

template <typename Scalar>
Vector3<Scalar> Rotation<Scalar>::rotate(Vector3<Scalar> vector){
    return Vector3<Scalar>(
        this->elements[0][0]*vector[0] + this->elements[0][1]*vector[1] + this->elements[0][2]*vector[2],
        this->elements[1][0]*vector[0] + this->elements[1][1]*vector[1] + this->elements[1][2]*vector[2],
        this->elements[2][0]*vector[0] + this->elements[2][1]*vector[1] + this->elements[2][2]*vector[2]
    );
}

template <typename Scalar>
Rotation<Scalar> Rotation<Scalar>::transpose(){
    Scalar transposed_elements[3][3];
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            transposed_elements[i][j] = this->elements[j][i];
        }
    }
    return Rotation<Scalar>(transposed_elements);
}

// Explicitly Instantiate floats and doubles:
template struct Rotation<float>;
template struct Rotation<double>;