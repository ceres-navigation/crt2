#ifndef __TRANSFORM_NODE_H_
#define __TRANSFORM_NODE_H_

#include "primitives/ray.hpp"
#include "utils/vector.hpp"

template <typename Scalar>
class TransformNode {
    public: 
        virtual void apply(Ray<Scalar>& ray);
        virtual void invert(Ray<Scalar>& ray);
        virtual void apply(Vector3<Scalar>& vector);
        virtual void invert(Vector3<Scalar>& vector);
};

#endif