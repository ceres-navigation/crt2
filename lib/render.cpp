#include "render.hpp"

#include "acceleration/scene.hpp"
#include "cameras/camera.hpp"
#include "lights/light.hpp"

#include <cmath>
#include <iomanip>
#include <vector>
#include <iostream>
#include <chrono>
#include <algorithm>

template <typename Scalar>
std::vector<uint8_t> render(Scene<Scalar>&scene, Camera<Scalar>& camera, std::vector<Light<Scalar>*> lights){

    // THINGS TO MOVE OUTSIDE OF FUNCTION:
    int tile_size = 16;
    bool print_statements = false;
    int max_samples = 1;

    if (print_statements){
        std::cout << "Rendering...\n";
    }
    auto start = std::chrono::high_resolution_clock::now();

    // Initialize the image:
    size_t width  = (size_t) floor(camera.sensor->get_resolution_h());
    size_t height = (size_t) floor(camera.sensor->get_resolution_v());
    auto pixels = std::make_unique<float[]>(4 * width * height);

    // Loop over tiles:
    for( int u = 0; u < camera.sensor->get_resolution_h(); u+=tile_size) {
        for (int v = 0; v < camera.sensor->get_resolution_v(); v+=tile_size){

            // Loop over pixels in current tile:
            for (int x = 0; x < tile_size; x++){
                for (int y = 0; y < tile_size; y++){

                    size_t index = 4 * (width * (v+y) + (u+x));
                    Vector3<Scalar> pixel_radiance(0);

                    // Loop over samples per pixel:
                    for  (int sample = 0; sample < max_samples; ++sample) {
                        
                        // Generate a random sample:
                        // auto ray = camera.pixel_to_ray(u+x,v+y);
                        // bvh::Ray<Scalar> ray;
                        // auto x_rand = distr(eng);
                        // auto y_rand = distr(eng);
                        // if (max_samples == 1) {
                        //     ray = camera->pixel_to_ray(u+x, v+y);
                        // }
                        // else {
                        //     ray = camera->pixel_to_ray(u+x + x_rand, v+y + y_rand);
                        // }

                        auto ray = camera.pixel_to_ray(u+x, v+y);

                        // Perform path tracing operation:
                        // TODO!

                        // This is a simple place holder:
                        scene.Intersect( ray );
                        if (ray.hit.t < std::numeric_limits<Scalar>::max()) {
                            pixel_radiance[0] = 255;
                            pixel_radiance[1] = 255;
                            pixel_radiance[2] = 255;
                        }

                        // Run adaptive sampling:
                        // TODO!
                    }

                    // Store the pixel intensity:
                    pixels[index    ] = pixel_radiance[0];
                    pixels[index + 1] = pixel_radiance[1];
                    pixels[index + 2] = pixel_radiance[2];
                    pixels[index + 3] = 1;
                }
            }
        }
    }

    // Construct output image:
    std::vector<uint8_t> image;
    image.reserve(4*width*height);

    for (size_t j = 0; j < 4*width*height; j++) {
        image.push_back((uint8_t) std::clamp(pixels[j] * 256, 0.0f, 255.0f));
    }

    for(unsigned y = 0; y < height; y++) {
        for(unsigned x = 0; x < width; x++) {
            size_t i = 4 * (width * y + x);
            image[4 * width * y + 4 * x + 0] = (uint8_t) std::clamp(pixels[i+0] * 256, 0.0f, 255.0f);
            image[4 * width * y + 4 * x + 1] = (uint8_t) std::clamp(pixels[i+1] * 256, 0.0f, 255.0f);
            image[4 * width * y + 4 * x + 2] = (uint8_t) std::clamp(pixels[i+2] * 256, 0.0f, 255.0f);
            image[4 * width * y + 4 * x + 3] = (uint8_t) std::clamp(pixels[i+3] * 256, 0.0f, 255.0f);
        }
    }

    if (print_statements) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "    Rendered in " << duration.count() << " milliseconds\n";
    }

    return image;
};

template std::vector<uint8_t> render<float>(Scene<float>& scene, Camera<float>& camera, std::vector<Light<float>*> lights);
template std::vector<uint8_t> render<double>(Scene<double>& scene, Camera<double>& camera, std::vector<Light<double>*> lights);