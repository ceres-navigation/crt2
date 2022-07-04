#include <vector>

#include "vector_math/vector.hpp"
#include "sensors/simple_sensor.hpp"

#include "cameras/camera.hpp"
#include "cameras/simple_camera.hpp"

#include "lights/light.hpp"
#include "lights/point_light.hpp"

#include "primitives/triangle.hpp"

#include <iostream>

// TODO: Make this configurable:
using Scalar = double;

void print_vector(Vector3<Scalar> v){
    std::cout << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]\n";
}

void render(Camera<Scalar> &camera, Light<Scalar> &light, Triangle<Scalar> triangles[1000]){
    //Simple ray tracing:
    auto res_h = camera.sensor->get_resolution_h();
    auto res_v = camera.sensor->get_resolution_v();

    // Loop through all pixels:
    for (int u = 0; u < res_h; u++){
        for (int v = 0; v < res_v; v++){

            // Generate a ray:
            Ray<Scalar> ray = camera.pixel_to_ray(u,v);

            // Trace ray against all triangles:
            for (int i = 0; i < 1000; i++){
                intersect_triangle<Scalar>(ray, triangles[i]);
            }

            std::cout << "Ray Origin: ";
            print_vector(ray.origin);
            std::cout << "Ray Direction: ";
            print_vector(ray.direction);
            std::cout << "Triangle v0: ";
            print_vector(triangles[0].vertex0);
            std::cout << "Triangle v1: ";
            print_vector(triangles[0].vertex1);
            std::cout << "Triangle v2: ";
            print_vector(triangles[0].vertex2);

            std::cout << ray.hit.t << "\n";

            std::cout << (ray.hit.t != std::numeric_limits<Scalar>::max()) << "\n";
            
            // Format an output image:

        }
    }
}


int main(){
    // Define sensor:
    Scalar resolution[2] = {1,1};
    Scalar size[2] = {30,30};
    SimpleSensor<Scalar> sensor(resolution, size);

    // Define camera:
    SimpleCamera<Scalar> camera(30, sensor, true);
    camera.set_position(0,0,-18);

    std::cout << camera.center[0] << ", " << camera.center[1] << "\n";

    // Define a simple light:
    PointLight<Scalar> light(1);

    // Define some random triangles:
    Triangle<Scalar> triangles[1000];
    for (int i = 0; i < 1000; i++){
        triangles[i].vertex0 = Vector3<Scalar>(-5,-5,0);
        triangles[i].vertex1 = Vector3<Scalar>(0,5,0);
        triangles[i].vertex2 = Vector3<Scalar>(5,-5,0);
    }

    // Ray trace:
    render(camera, light, triangles);
    return 0;
}