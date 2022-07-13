#include "scene.hpp"

#include <vector>
#include <iostream>
#include <chrono>
#include <random>
#include <algorithm>
#include <iomanip>

#include "cameras/camera.hpp"
#include "lights/light.hpp"

#include "primitives/geometry.hpp"
#include "primitives/ray.hpp"

template <typename Scalar>
Scene<Scalar>::Scene( Geometry<Scalar>* geometryList, uint N ) {
    // Store all BLAS:
    blas = new BVH<Scalar>[N];
    for (uint i = 0; i < N; i++) {
        blas[i] = *geometryList[i].bvh;
    }

    // copy a pointer to the array of bottom level accstructs
    blasCount = N;

    // allocate Scene nodes
    tlasNode = new TLASNode<Scalar>[2*N];
    nodesUsed = 0;
}

template <typename Scalar>
uint Scene<Scalar>::FindBestMatch( uint* list, uint N, uint A ) {
	// find BLAS B that, when joined with A, forms the smallest AABB
	Scalar smallest = std::numeric_limits<Scalar>::max();
	uint bestB = -1;
	for (uint B = 0; B < N; B++) {
        if (B != A) {
            Vector3<Scalar> bmax = max( tlasNode[list[A]].aabbMax, tlasNode[list[B]].aabbMax );
            Vector3<Scalar> bmin = min( tlasNode[list[A]].aabbMin, tlasNode[list[B]].aabbMin );
            Vector3<Scalar> e = bmax - bmin;
            Scalar surfaceArea = e[0] * e[1] + e[1] * e[2] + e[2] * e[0];
            if (surfaceArea < smallest){
                smallest = surfaceArea;
                bestB = B;
            }
        }
    }
	return bestB;
}

template <typename Scalar>
void Scene<Scalar>::Build() {
    // assign a Scene leaf node to each BLAS
    uint* nodeIdx = new uint[blasCount];
    uint nodeIndices = blasCount;
    nodesUsed = 1;
    for (uint i = 0; i < blasCount; i++) {
        nodeIdx[i] = nodesUsed;
        tlasNode[nodesUsed].aabbMin = blas[i].bounds.bmin;
        tlasNode[nodesUsed].aabbMax = blas[i].bounds.bmax;
        tlasNode[nodesUsed].blas_id = i;
        tlasNode[nodesUsed++].leftRight = 0;
    }

    // use agglomerative clustering to build the Scene
	uint A = 0;
    uint B = FindBestMatch( nodeIdx, nodeIndices, A );
	while (nodeIndices > 1) {
		uint C = FindBestMatch( nodeIdx, nodeIndices, B );
		if (A == C) {
			uint nodeIdxA = nodeIdx[A], nodeIdxB = nodeIdx[B];
			TLASNode<Scalar>& nodeA = tlasNode[nodeIdxA];
			TLASNode<Scalar>& nodeB = tlasNode[nodeIdxB];
			TLASNode<Scalar>& newNode = tlasNode[nodesUsed];
			newNode.leftRight = nodeIdxA + (nodeIdxB << 16);
			newNode.aabbMin = min( nodeA.aabbMin, nodeB.aabbMin );
			newNode.aabbMax = max( nodeA.aabbMax, nodeB.aabbMax );
			nodeIdx[A] = nodesUsed++;
			nodeIdx[B] = nodeIdx[nodeIndices - 1];
			B = FindBestMatch( nodeIdx, --nodeIndices, A );
		}
		else {
            A = B;
            B = C;
        }
	}
	tlasNode[0] = tlasNode[nodeIdx[A]];

	delete[] nodeIdx;
}

template <typename Scalar>
void Scene<Scalar>::Intersect( Ray<Scalar>& ray ) {
	TLASNode<Scalar>* node = &tlasNode[0], * stack[64];
	uint stackPtr = 0;
	while (1) {
        if (node->isLeaf()) {
			blas[node->blas_id].Intersect( ray );
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		TLASNode<Scalar>* child1 = &tlasNode[node->leftRight & 0xffff];
		TLASNode<Scalar>* child2 = &tlasNode[node->leftRight >> 16];

		Scalar dist1 = intersect_aabb( ray, child1->aabbMin, child1->aabbMax );
		Scalar dist2 = intersect_aabb( ray, child2->aabbMin, child2->aabbMax );

		if (dist1 > dist2) { 
            std::swap( dist1, dist2 ); 
            std::swap( child1, child2 );
        }
		if (dist1 == std::numeric_limits<Scalar>::max()) {
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else {
			node = child1;
			if (dist2 != std::numeric_limits<Scalar>::max()) stack[stackPtr++] = child2;
		}
	}
};

template<typename Scalar>
std::vector<uint8_t> Scene<Scalar>::render(Camera<Scalar>& camera, std::vector<Light<Scalar>*> lights){

    // THINGS TO MOVE OUTSIDE OF FUNCTION:
    int tile_size = 16;
    int max_samples = 10;
	bool print_statements = false;

    if (print_statements){
        std::cout << "Rendering...\n";
    }
    auto start = std::chrono::high_resolution_clock::now();

    // Initialize random number generator:
    std::random_device rd;
    std::minstd_rand eng(rd());
    std::uniform_real_distribution<Scalar> distr(-0.5, 0.5);
    std::uniform_real_distribution<Scalar> dist1(0.0, 1.0);

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
                        Ray<Scalar> ray;
                        if (max_samples == 1) {
                            ray = camera.pixel_to_ray(u+x, v+y);
                        }
                        else {
                            auto x_rand = distr(eng);
                            auto y_rand = distr(eng);
                            ray = camera.pixel_to_ray(u+x + x_rand, v+y + y_rand);
                        }

                        // Intersect ray with scene:
                        Intersect( ray );

                        // Skip illumination computation if no hit:
                        if (ray.hit.t == std::numeric_limits<Scalar>::max()) {
                            break;
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
                            Intersect( light_ray );

                            // If ray is not obstructed, evaluate the illumination model:
                            if (light_ray.hit.t > light_distance){// || light_ray.hit.t < tol) {
                                // SIMPLE LAMBERTIAN BRDF:
                                Scalar L_dot_N = dot(light_ray.direction, normal);
                                Scalar intensity = light->get_intensity(intersect_point);
                                pixel_radiance = pixel_radiance + Vector3<Scalar>(L_dot_N*intensity);

                                // std::cout << L_dot_N*intensity << "\n";
                                // TODO USE THE MATERIAL POINTER:
                                
                            }
                        }

                        // Run adaptive sampling, and bounce ray cast:
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

// Explicitly Instantiate floats and doubles:
template class Scene<float>;
template class Scene<double>;