#include "utils/rotation.hpp"

// Basis rotations:
template<typename Scalar>
Rotation<Scalar> rotation_X(Scalar x);

template<typename Scalar>
Rotation<Scalar> rotation_Y(Scalar y);

template<typename Scalar>
Rotation<Scalar> rotation_Z(Scalar z);


// Composite euler angles:
template <typename Scalar>
Rotation<Scalar> XYZ_euler(Scalar x, Scalar y, Scalar z);

template <typename Scalar>
Rotation<Scalar> XZY_euler(Scalar x, Scalar y, Scalar z);

template <typename Scalar>
Rotation<Scalar> YXZ_euler(Scalar x, Scalar y, Scalar z);

template <typename Scalar>
Rotation<Scalar> YZX_euler(Scalar x, Scalar y, Scalar z);

template <typename Scalar>
Rotation<Scalar> ZXY_euler(Scalar x, Scalar y, Scalar z);

template <typename Scalar>
Rotation<Scalar> ZYX_euler(Scalar x, Scalar y, Scalar z);