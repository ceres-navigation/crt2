#ifndef __TRANSLATE_NODE_H_
#define __TRANSLATE_NODE_H_

#include "acceleration/transform_node.hpp"

#include "primitives/ray.hpp"
#include "utils/vector.hpp"

template <typename Scalar>
class TranslateNode : public TransformNode<Scalar> {
    public:
        Vector3<Scalar> translation;

        TranslateNode(Vector3<Scalar>);

        void apply(Ray<Scalar>& ray);
        void invert(Ray<Scalar>& ray);

        void apply(Vector3<Scalar>& vector);
        void invert(Vector3<Scalar>& vector);
};

template <typename Scalar>
TranslateNode<Scalar>::TranslateNode(Vector3<Scalar> translation){
    this->translation = translation;
};

template <typename Scalar>
void TranslateNode<Scalar>::apply(Ray<Scalar>& ray){
    ray.origin = ray.origin + this->translation;
};

template <typename Scalar>
void TranslateNode<Scalar>::invert(Ray<Scalar>& ray){
    ray.origin = ray.origin - this->translation;
}

template <typename Scalar>
void TranslateNode<Scalar>::apply(Vector3<Scalar>& vector){
    vector = vector + this->translation;
};

template <typename Scalar>
void TranslateNode<Scalar>::invert(Vector3<Scalar>& vector){
    vector = vector - this->translation;
}

#endif