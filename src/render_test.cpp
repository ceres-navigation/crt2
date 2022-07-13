#include <iostream>
#include <random>
#include <fstream> // Needed for writing ppm image

#include "lodepng.h"

#include "utils/vector.hpp"
#include "sensors/simple_sensor.hpp"

#include "cameras/camera.hpp"
#include "cameras/simple_camera.hpp"

#include "lights/light.hpp"
#include "lights/point_light.hpp"

#include "primitives/triangle.hpp"
#include "primitives/aabb.hpp"
#include "primitives/geometry.hpp"

#include "scene.hpp"

#include <chrono>

using Scalar = float;

int main(){
    // Define sensor:
    Scalar resolution[2] = {500,500};
    // Scalar resolution[2] = {100,100};
    Scalar size[2] = {30,30};
    SimpleSensor<Scalar> sensor(resolution, size);

    // Define camera:
    SimpleCamera<Scalar> camera(30, sensor, true);
    camera.set_position(0,0,-5);

    // Define a simple light:
    std::vector<Light<Scalar>*> lights;
    lights.push_back(new PointLight<Scalar>(10));
    lights[0]->set_position(10,0,5);

    // Load geometries:
    int N = 7;
    Geometry<Scalar>* geometries = new Geometry<Scalar>[N];
    for (int i = 0; i < N; i++) {
        geometries[i].read_obj("../suzanne.obj");//, "obj");
        geometries[i].build_bvh();
        geometries[i].set_position(Vector3<Scalar>(0,2.5*i - 7,9));
    }

    // Create the scene:
    auto scene = Scene(geometries, N);
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
    // std::string filename = "frame_bvh.ppm";
    // std::ofstream ppm2(filename, std::ios::binary);
    // ppm2 << "P5\n" << sensor.get_resolution_h() << ' ' << sensor.get_resolution_v() << "\n255\n";
    // ppm2.write(reinterpret_cast<char *>(image), camera.sensor->get_resolution_h() * camera.sensor->get_resolution_v() * sizeof(uint8_t));
    // ppm2.flush();

    return 0;
}