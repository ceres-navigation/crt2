#include <vector>

#include "acceleration/bvh.hpp"

#include "vector_math/vector.hpp"
#include "sensors/simple_sensor.hpp"

#include "cameras/camera.hpp"
#include "cameras/simple_camera.hpp"

#include "lights/light.hpp"
#include "lights/point_light.hpp"

#include "primitives/triangle.hpp"

#include "geometry.hpp"

#include <iostream>

// TODO: Make this configurable:
using Scalar = double;


#include <random> // Needed for generating random numbers

#include <fstream> // Needed for writing ppm image

// Helper function to print vector3 (if needed):
void print_vector(Vector3<Scalar> v){
    std::cout << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]\n";
}

void render(Camera<Scalar> &camera, Light<Scalar> &light, Geometry<Scalar> &geometry){
    //Simple ray tracing:
    auto res_h = camera.sensor->get_resolution_h();
    auto res_v = camera.sensor->get_resolution_v();

    uint8_t  image[(int) res_h][(int) res_v] = {0};

    // Loop through all pixels:
    for (int u = 0; u < res_h; u++){
        for (int v = 0; v < res_v; v++){

            // Generate a ray:
            Ray<Scalar> ray = camera.pixel_to_ray(u,v);

            // Trace against bvh:
            geometry.intersect(ray);

            // Format an output image:
            if (ray.hit.t < std::numeric_limits<Scalar>::max()) {
                image[u][v] = 255;
            }
        }
    }

    // Write image to file:
    std::string filename = "frame.ppm";
    std::ofstream ppm(filename, std::ios::binary);
    ppm << "P5\n" << res_h << ' ' << res_v << "\n255\n";
    ppm.write(reinterpret_cast<char *>(image), res_h * res_v * sizeof(uint8_t));
    ppm.flush();
}

int main(){
    // Define sensor:
    Scalar resolution[2] = {1000,1000};
    Scalar size[2] = {30,30};
    SimpleSensor<Scalar> sensor(resolution, size);

    // Define camera:
    SimpleCamera<Scalar> camera(30, sensor, true);
    camera.set_position(0,0,-5);

    // Define a simple light:
    PointLight<Scalar> light(1);

    // Load geometry and construct BVH:
    Geometry<Scalar> geometry("cube");
    // geometry.read_obj("../suzanne.obj");
    // geometry.read_obj("../cube.obj");
    geometry.read_obj("../random.obj");

    geometry.build_bvh();

    // Ray trace:
    std::cout << "Beginning render...\n";
    render(camera, light, geometry);

    return 0;
}