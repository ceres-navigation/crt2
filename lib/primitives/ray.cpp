#include "vector_math/vector.hpp"
#include "primitives/ray.hpp"

template <typename Scalar>
Ray<Scalar>::Ray() {
    this -> origin = Vector3<Scalar>(0,0,0);
    this -> direction = Vector3<Scalar>(0,0,0); 
    this -> t = std::numeric_limits<Scalar>::max();
};

template <typename Scalar>
Ray<Scalar>::Ray(Vector3<Scalar> origin, Vector3<Scalar> direction) {
    this -> origin = origin;
    this -> direction = direction;
    this -> t = std::numeric_limits<Scalar>::max();
};

template <typename Scalar>
Ray<Scalar>::Ray(Vector3<Scalar> origin, Vector3<Scalar> direction, Scalar t_max) {
    this -> origin = origin;
    this -> direction = direction;
    this -> t = t_max;
};


// Explicitly Instantiate floats and doubles:
template struct Ray<float>;
template struct Ray<double>;