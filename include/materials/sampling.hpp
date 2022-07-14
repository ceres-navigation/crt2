#ifndef __SAMPLING_H_
#define __SAMPLING_H_

#include "utils/vector.hpp"

template <typename Scalar>
Vector3<Scalar> cosine_importance(Vector3<Scalar> normal, Scalar r1, Scalar r2);

#endif 