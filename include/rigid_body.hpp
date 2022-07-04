#ifndef __RIGID_BODY_H
#define __RIGID_BODY_H

#include "vector_math/vector.hpp"
#include "vector_math/rotation.hpp"

template <typename Scalar>
class RigidBody {

    public:
        Vector3<Scalar> position;
        Rotation<Scalar> rotation;

        RigidBody();

        RigidBody(Vector3<Scalar> position);

        RigidBody(Rotation<Scalar> rotation);

        RigidBody(Vector3<Scalar> position, Rotation<Scalar> rotation);

        void set_position(Vector3<Scalar> new_position);

        void set_position(Scalar x, Scalar y, Scalar z);

        void set_rotation(Rotation<Scalar> new_rotation);

        void set_pose(Vector3<Scalar> new_position, Rotation<Scalar> new_rotation);
};

#endif