#include "rigid_body.hpp"
#include "vector_math/vector.hpp"
#include "vector_math/rotation.hpp"

template <typename Scalar>
RigidBody<Scalar>::RigidBody(){
    this -> position = Vector3<Scalar>();
    this -> rotation = Rotation<Scalar>();
};

template <typename Scalar>
RigidBody<Scalar>::RigidBody(Vector3<Scalar> position){
    this -> position = position;
    this -> rotation = Rotation<Scalar>();
};

template <typename Scalar>
RigidBody<Scalar>::RigidBody(Rotation<Scalar> rotation){
    this -> position = Vector3<Scalar>();
    this -> rotation = rotation;
};

template <typename Scalar>
RigidBody<Scalar>::RigidBody(Vector3<Scalar> position, Rotation<Scalar> rotation){
    this -> position = position;
    this -> rotation = rotation;
};

template <typename Scalar>
void RigidBody<Scalar>::set_position(Vector3<Scalar> new_position) {
    this -> position = new_position;
};

template <typename Scalar>
void RigidBody<Scalar>::set_rotation(Rotation<Scalar> new_rotation){
    this -> rotation = new_rotation;
};

template <typename Scalar>
void RigidBody<Scalar>::set_pose(Vector3<Scalar> new_position, Rotation<Scalar> new_rotation){
    this -> position = new_position;
    this -> rotation = new_rotation;
};


// Explicitly Instantiate floats and doubles:
template class RigidBody<float>;
template class RigidBody<double>;