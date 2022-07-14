#include "materials/sampling.hpp"
#include "math.h"

template <typename Scalar>
Vector3<Scalar> cosine_importance(Vector3<Scalar> normal, Scalar r1, Scalar r2) {
    // This implementation is based on Malley's method:
    // NOTE: Assumes left-handed normal!!!
    // Generate a random sample:
    Scalar r = std::sqrt(r1);
    Scalar theta = r2*2*3.1415926;
    Scalar x = -r*std::cos(theta);
    Scalar y = -r*std::sin(theta);
    Scalar z = -std::sqrt(std::max(0.0, 1.0 - x*x - y*y));

    // Transform back into world coordinates:
    Vector3<Scalar> Nx;
    // Per PBR, need handle two cases
    // not sure what they are, but avoids div by zero
    if (std::abs(normal[0]) > std::abs(normal[1])) {
        Nx = Vector3<Scalar>(normal[2],0.0,-normal[0])*Scalar(1.0/std::sqrt(normal[0]*normal[0] + normal[2]*normal[2]));
    } else {
        Nx = Vector3<Scalar>(0, normal[2],-normal[1])*Scalar(1.0/std::sqrt(normal[1]*normal[1] + normal[2]*normal[2]));
    }

    Vector3<Scalar> Ny = cross(normal, Nx);

    auto dir = Nx*x + Ny*y + normal*z;
    return dir;
};

// Explicitly Instantiate floats and doubles:
template Vector3<float> cosine_importance<float>(Vector3<float> normal, float r1, float r2);
template Vector3<double> cosine_importance<double>(Vector3<double> normal, double r1, double r2);