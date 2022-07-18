#include <vector>
#include <iostream>
#include <chrono>
#include <random>
#include <algorithm>
#include <iomanip>

#include "scene.hpp"
#include "geometry.hpp"

#include "cameras/camera.hpp"
#include "lights/light.hpp"

#include "primitives/ray.hpp"

#include "physics/spectral_radiance.hpp"

#include "path_tracing/backward.hpp"

#include <oneapi/tbb.h>
#include <oneapi/tbb/mutex.h>
#include <oneapi/tbb/rw_mutex.h>
typedef tbb::mutex mutex_t;
typedef tbb::rw_mutex rw_mutex_t;
#include <tbb/parallel_for.h>

// TEMPORARY REMOVE THIS:
#include <tbb/global_control.h>

template <typename Scalar>
Scene<Scalar>::Scene( std::vector<Geometry<Scalar>*> geometry_list) {
    // Store all BLAS:
    blas = new BVH<Scalar>*[geometry_list.size()];
    for (uint i = 0; i < geometry_list.size(); i++) {
        blas[i] = geometry_list[i]->bvh;
    }

    // copy a pointer to the array of bottom level accstructs
    blasCount = geometry_list.size();

    // allocate Scene nodes
    tlasNode = new TLASNode<Scalar>[2*geometry_list.size()];
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
		tlasNode[nodesUsed].aabbMin = blas[i]->bounds_world.bmin;
        tlasNode[nodesUsed].aabbMax = blas[i]->bounds_world.bmax;
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
			blas[node->blas_id]->Intersect( ray );
			if (stackPtr == 0) {
                break; 
            } else {
                node = stack[--stackPtr];
            }
			continue;
		}
		TLASNode<Scalar>* child1 = &tlasNode[node->leftRight & 0xffff];
		TLASNode<Scalar>* child2 = &tlasNode[node->leftRight >> 16];

		Scalar dist1 = intersect_aabb( ray, child1->aabbMin, child1->aabbMax );
		Scalar dist2 = intersect_aabb( ray, child2->aabbMin, child2->aabbMax );

        // Swap order of traversal if child2 is closer:
		if (dist1 > dist2) { 
            std::swap( dist1, dist2 ); 
            std::swap( child1, child2 );
        }

        // If the closest one is still infinite, skip:
		if (dist1 == std::numeric_limits<Scalar>::max()) {
			if (stackPtr == 0){
                break;
            } else {
                node = stack[--stackPtr];
            }
		}
		else {
			node = child1;
			if (dist2 != std::numeric_limits<Scalar>::max()){
                stack[stackPtr++] = child2;
            }
		}
	}
};

template<typename Scalar>
std::vector<uint8_t> Scene<Scalar>::render(Camera<Scalar>& camera, std::vector<Light<Scalar>*> lights){

    // THINGS TO MOVE OUTSIDE OF FUNCTION:
    uint num_bounces = 0;
    size_t tile_size = 64;
    uint min_samples = 0;
    uint max_samples = 1;
    Scalar noise_threshold = 0.1;
	bool print_statements = true;

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

    // auto pixels = std::make_unique<float[]>(4 * width * height);
    float pixels[4 * width * height] = {0};

    // Use sensor sensitivity and pixel size:
    // Scalar radiance_to_pixel = camera.sensor->radiance_to_pixel();
    const Scalar radiance_to_pixel = (camera.sensor->size[0]/width)*(camera.sensor->size[1]/height);

    //TEMPORARY REMOVE THIS!
    // tbb::global_control c(tbb::global_control::max_allowed_parallelism, 1);

    // Loop over tiles:
    int tile_number = 0;
    for( size_t u = 0; u < width; u+=tile_size) {
        for (size_t v = 0; v < height; v+=tile_size){

            // Determine if the tile reaches the end of the image:
            size_t tile_width = tile_size;
            size_t tile_height = tile_size;
            if (width-u < tile_size){
                tile_width = width-u;
            }
            if (height-v < tile_size){
                tile_height = height-v;
            }

            // Loop over pixels in current tile:
            tbb::parallel_for( tbb::blocked_range2d<size_t>(0, tile_height, 0, tile_width), [=, &camera, &distr, &eng, &lights, &pixels](const tbb::blocked_range2d<size_t>& r) {
            for (size_t x = r.cols().begin(), x_end=r.cols().end(); x<x_end; x++){
                for (size_t y = r.rows().begin(), y_end=r.rows().end(); y<y_end; y++){
            // for (size_t x = 0; x < tile_width; x++){
            //     for (size_t y = 0; y < tile_height; y++){
                    size_t index = 4 * (width * (v+y) + (u+x));
                    SpectralRadiance<Scalar> pixel_radiance(0);

                    // Loop over samples per pixel:
                    for (uint sample = 1; sample < max_samples+1; sample++) {
                        
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

                        // Evaluate path tracing:
                        Vector3<Scalar> path_radiance(0);
                        backward_trace<Scalar>(this, ray, lights, num_bounces, path_radiance);

                        // Run adaptive sampling:
                        SpectralRadiance<Scalar> rad_contrib = (path_radiance - pixel_radiance)*((Scalar) 1/sample);
                        pixel_radiance += rad_contrib;
                        if (sample >= min_samples) {
                            Scalar noise = length(rad_contrib);
                            if (noise < noise_threshold) {
                                break;
                            }
                        }
                    }

                    // Store the pixel intensity:
                    if (index >= 4*width*height){
                        std::cout << "    index >= 1400000 (ERROR)\n";
                    }
                    pixels[index    ] = pixel_radiance[0]*radiance_to_pixel;
                    pixels[index + 1] = pixel_radiance[1]*radiance_to_pixel;
                    pixels[index + 2] = pixel_radiance[2]*radiance_to_pixel;
                    pixels[index + 3] = 1;
                }
            }
            }); // UNCOMMENT THIS OUT FOR TBB
            std::cout << "Tile " << tile_number << " rendered...\n";
            tile_number++;
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