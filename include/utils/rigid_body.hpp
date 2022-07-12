#ifndef __RIGID_BODY_H
#define __RIGID_BODY_H

#include "utils/vector.hpp"
#include "utils/rotation.hpp"

template <typename Scalar>
class RigidBody {

    public:
        Scalar scale;
        Vector3<Scalar> position;
        Rotation<Scalar> rotation;

        RigidBody();

        RigidBody(Scalar scale);

        RigidBody(Vector3<Scalar> position);

        RigidBody(Rotation<Scalar> rotation);

        RigidBody(Vector3<Scalar> position, Rotation<Scalar> rotation);

        RigidBody(Scalar scale, Vector3<Scalar> position, Rotation<Scalar> rotation);

        void set_scale(Scalar new_scale);

        void set_position(Vector3<Scalar> new_position);

        void set_position(Scalar x, Scalar y, Scalar z);

        void set_rotation(Rotation<Scalar> new_rotation);

        void set_pose(Vector3<Scalar> new_position, Rotation<Scalar> new_rotation);

        void set_transform(Scalar new_scale, Vector3<Scalar> new_position, Rotation<Scalar> new_rotation);
};

#endif