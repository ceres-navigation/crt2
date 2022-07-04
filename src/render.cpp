#include <vector>

#include "vector_math/vector.hpp"
#include "sensors/simple_sensor.hpp"

#include "cameras/camera.hpp"
#include "cameras/simple_camera.hpp"

#include "lights/light.hpp"
#include "lights/point_light.hpp"

#include "primitives/triangle.hpp"

#include <iostream>

using Scalar = double;
void render(Camera<Scalar> &camera, Light<Scalar> &light, std::vector<Triangle<Scalar>> triangles){
    //Simple ray tracing:
}


int main(){
    // Define sensor:
    Scalar resolution[2] = {500,500};
    Scalar size[2] = {30,30};
    SimpleSensor<Scalar> sensor(resolution, size);

    // Define camera:
    SimpleCamera<Scalar> camera(30, sensor, true);

    // Define a simple light:
    PointLight<Scalar> light(1);

    // Define triangles:
    std::vector<Triangle<Scalar>> triangles;

    // Ray trace:
    render(camera, light, triangles);
    return 0;
}