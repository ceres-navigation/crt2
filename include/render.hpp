#include "acceleration/scene.hpp"
#include "cameras/camera.hpp"
#include "lights/light.hpp"

template <typename Scalar>
std::vector<uint8_t> render(Scene<Scalar>&scene, Camera<Scalar>& camera, std::vector<Light<Scalar>*> lights);