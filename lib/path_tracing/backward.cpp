#include "path_tracing/backward.hpp"

#include "physics/spectral_radiance.hpp"

#include "scene.hpp"
#include "primitives/ray.hpp"
#include "lights/light.hpp"

template<typename Scalar>
void backward_trace(Scene<Scalar>* scene, Ray<Scalar>& ray, std::vector<Light<Scalar>*> &lights, uint num_bounces, SpectralRadiance<Scalar> &path_radiance){
    for (uint bounce = 0; bounce < num_bounces+1; bounce++){
        // Intersect ray with scene:
        scene->Intersect( ray );

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
        // auto interp_uv = u*tridata.uv1 + v*tridata.uv2 + (Scalar(1.0)-u-v)*tridata.uv0;

        // Cast ray towards the light source:
        for (auto light: lights){
            // Scalar tol = 0.0001;
            intersect_point = intersect_point + normal* (Scalar) 0.0001;
            Ray<Scalar> light_ray = light->sample_ray(intersect_point);
            Scalar light_distance = light_ray.t;
            scene->Intersect( light_ray );

            // If ray is not obstructed, evaluate the illumination model:
            if (light_ray.hit.t > light_distance){// || light_ray.hit.t < tol) {
                // SIMPLE LAMBERTIAN BRDF:
                Scalar L_dot_N = dot(light_ray.direction, normal);
                Scalar intensity = light->get_intensity(intersect_point);
                path_radiance = path_radiance + SpectralRadiance<Scalar>(L_dot_N*intensity);

                // TODO USE THE MATERIAL POINTER:
                
            }
        }

        // Exit if the max number of bounces is set:
        if (bounce == num_bounces){
            return;
        }

        // Cast bounce ray:
        Vector3<Scalar> bounce_direction(0);
        ray = Ray<Scalar>(intersect_point, bounce_direction);
    }
};

// Explicitly Instantiate floats and doubles:
template void backward_trace<float>(Scene<float>* scene, Ray<float>& ray, std::vector<Light<float>*> &lights, uint num_bounces, Vector3<float> &pixel_radiance);
template void backward_trace<double>(Scene<double>* scene, Ray<double>& ray, std::vector<Light<double>*> &lights, uint num_bounces, Vector3<double> &pixel_radiance);