#include <iostream>
#include <random>
#include <fstream> // Needed for writing ppm image

#include "lodepng.h"

#include "utils/vector.hpp"
#include "utils/rotation.hpp"
#include "utils/euler_angles.hpp"

#include "sensors/simple_sensor.hpp"

#include "cameras/camera.hpp"
#include "cameras/simple_camera.hpp"

#include "lights/light.hpp"
#include "lights/point_light.hpp"

#include "primitives/triangle.hpp"
#include "primitives/aabb.hpp"
#include "geometry.hpp"

#include "scene.hpp"

#include <chrono>

using Scalar = float;

int main(){
    // Define sensor:
    Scalar resolution[2] = {500,500};
    // Scalar resolution[2] = {100,100};
    Scalar size[2] = {36,36};
    SimpleSensor<Scalar> sensor(resolution, size);

    // Define camera:
    SimpleCamera<Scalar> camera(50, sensor, false);
    camera.set_position(4,-4,1.1);
    Rotation<Scalar> rotation = XYZ_euler<Scalar>(78,0,44);
    camera.set_rotation(rotation);

    // Define a simple light:
    std::vector<Light<Scalar>*> lights;
    lights.push_back(new PointLight<Scalar>(1));
    lights[0]->set_position(0,-3,0);

    std::vector<Geometry<Scalar>*> geometries;
    geometries.push_back(new Geometry<Scalar>("../suzanne.obj", "obj"));
    geometries[0]->set_position(Vector3<Scalar>(1,0,0));
    geometries[0]->set_rotation(ZXY_euler<Scalar>(11,83,22));

    // Create the scene:
    auto scene = Scene<Scalar>(geometries);
    scene.Build();

    // Render:
    auto image = scene.render(camera,lights);

    // Save the image:
    size_t width  = (size_t) floor(camera.sensor->get_resolution_h());
    size_t height = (size_t) floor(camera.sensor->get_resolution_v());
    unsigned error = lodepng::encode("frame.png", image, width, height);
    if(error) {
        std::cout << "PNG error " << error << ": "<< lodepng_error_text(error) << std::endl;
    }

    return 0;
}