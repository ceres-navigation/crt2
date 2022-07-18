#include "path_tracing/backward.hpp"

#include "physics/spectral_radiance.hpp"

#include "scene.hpp"
#include "primitives/ray.hpp"
#include "lights/light.hpp"

#include <math.h>

template<typename Scalar>
void backward_trace(Scene<Scalar>* scene, Ray<Scalar>& ray, std::vector<Light<Scalar>*> &lights, uint num_bounces,
                    SpectralRadiance<Scalar> &path_radiance, uint tile_number){
    
    Vector3<Scalar> weight(2*M_PI);
    for (uint bounce = 0; bounce < num_bounces+1; bounce++){
        // Intersect ray with scene:
        scene->Intersect( ray, tile_number);

        // Skip illumination computation if no hit:
        if (ray.hit.t == std::numeric_limits<Scalar>::max()) {
            return;
        }

        // Calculate intersection point:
        auto intersect_point = ray.origin + ray.hit.t*ray.direction;

        // Get the data for the triangle intersected:
        auto tridata = ray.hit.geometry->triangle_data[ray.hit.triIdx];

        // Calculate the normal:
        Vector3<Scalar> normal;
        if (ray.hit.geometry->smooth_shading){
            normal = normalize<Scalar>(ray.hit.u*tridata.vn1 +
                                        ray.hit.v*tridata.vn2 + 
                                        (Scalar(1.0)-ray.hit.u-ray.hit.v)*tridata.vn0);
        }
        else {
            normal = tridata.face_normal;
        }

        // Calculate the texture-space UV coordinates:
        Vector2<Scalar> interp_uv = ray.hit.u*tridata.uv1 + ray.hit.v*tridata.uv2 + (Scalar(1.0)-ray.hit.u-ray.hit.v)*tridata.uv0;

        // Cast ray towards the light source:
        SpectralRadiance<Scalar> bounce_color;
        for (auto light: lights){
            // Scalar tol = 0.0001;
            intersect_point = intersect_point + normal* (Scalar) 0.0001;
            Ray<Scalar> light_ray = light->sample_ray(intersect_point);
            Scalar light_distance = light_ray.t;
            scene->Intersect( light_ray, tile_number);

            // If ray is not obstructed, evaluate the illumination model:
            if (light_ray.hit.t > light_distance){// || light_ray.hit.t < tol) {
                bounce_color = ray.hit.geometry->material->get_color(light_ray, ray, normal, interp_uv);
                path_radiance = path_radiance + light->get_intensity(intersect_point)*bounce_color*weight;
            }
        }

        // Exit if the max number of bounces is set:
        if (bounce == num_bounces){
            return;
        }

        // Cast bounce ray:
        Vector3<Scalar> bounce_direction = ray.hit.geometry->material->bounce_ray(ray, normal, interp_uv);
        ray = Ray<Scalar>(intersect_point, bounce_direction);
        weight *= bounce_color;
    }
};

// Explicitly Instantiate floats and doubles:
template void backward_trace<float>(Scene<float>* scene, Ray<float>& ray, std::vector<Light<float>*> &lights, uint num_bounces, 
                                    SpectralRadiance<float> &pixel_radiance, uint tile_number);
template void backward_trace<double>(Scene<double>* scene, Ray<double>& ray, std::vector<Light<double>*> &lights, uint num_bounces,
                                     SpectralRadiance<double> &pixel_radiance, uint tile_number);