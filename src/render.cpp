#include <iostream>
#include <random>
#include <fstream> // Needed for writing ppm image

#include "vector_math/vector.hpp"
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
#include <vector>

using Scalar = float;

// TLAS CODE:
struct TLASNode {
    Vector3<Scalar> aabbMin;
    uint leftRight; // 2x16 bits
    Vector3<Scalar> aabbMax;
    uint blas_id;
    bool isLeaf() { return leftRight == 0; }
};

class TLAS {
    public:
        TLAS() = default;
        TLAS( Geometry<Scalar>* geometryList, uint N );
        void Build();
        void Intersect( Ray<Scalar>& ray );
        
    private:
        uint FindBestMatch( uint* list, uint N, uint A );
        TLASNode* tlasNode = 0;
        BVH<Scalar>* blas = 0;
        uint nodesUsed, blasCount;
};

TLAS::TLAS( Geometry<Scalar>* geometryList, uint N ) {
    // Store all BLAS:
    blas = new BVH<Scalar>[N];
    std::cout << "ADDING TO BLAS:\n";
    for (uint i = 0; i < N; i++) {
        blas[i] = *geometryList[i].bvh;
        std::cout << blas[i].bounds.bmin[0] << ", " << blas[i].bounds.bmin[1] << ", " << blas[i].bounds.bmin[2] << "\n";
        std::cout << blas[i].bounds.bmax[0] << ", " << blas[i].bounds.bmax[1] << ", " << blas[i].bounds.bmax[2] << "\n";
    }
    std::cout <<"\n";

    // copy a pointer to the array of bottom level accstructs
    blasCount = N;

    // allocate TLAS nodes
    tlasNode = new TLASNode[2*N];
    nodesUsed = 0;
}

uint TLAS::FindBestMatch( uint* list, uint N, uint A ) {
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

void TLAS::Build() {
    // assign a TLAS leaf node to each BLAS
    uint* nodeIdx = new uint[blasCount];
    uint nodeIndices = blasCount;
    nodesUsed = 1;
    for (uint i = 0; i < blasCount; i++) {
        nodeIdx[i] = nodesUsed;
        std::cout << blas[i].bounds.bmin[0] << ", " << blas[i].bounds.bmin[1] << ", " << blas[i].bounds.bmin[2] << "\n";
        std::cout << blas[i].bounds.bmax[0] << ", " << blas[i].bounds.bmax[1] << ", " << blas[i].bounds.bmax[2] << "\n";
        tlasNode[nodesUsed].aabbMin = blas[i].bounds.bmin;
        tlasNode[nodesUsed].aabbMax = blas[i].bounds.bmax;
        tlasNode[nodesUsed].blas_id = i;
        tlasNode[nodesUsed++].leftRight = 0;
    }

    // use agglomerative clustering to build the TLAS
	uint A = 0;
    uint B = FindBestMatch( nodeIdx, nodeIndices, A );
	while (nodeIndices > 1) {
		uint C = FindBestMatch( nodeIdx, nodeIndices, B );
		if (A == C) {
			uint nodeIdxA = nodeIdx[A], nodeIdxB = nodeIdx[B];
			TLASNode& nodeA = tlasNode[nodeIdxA];
			TLASNode& nodeB = tlasNode[nodeIdxB];
			TLASNode& newNode = tlasNode[nodesUsed];
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

void TLAS::Intersect( Ray<Scalar>& ray ) {
	TLASNode* node = &tlasNode[0], * stack[64];
	uint stackPtr = 0;
	while (1) {
        if (node->isLeaf()) {
			blas[node->blas_id].Intersect( ray );
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		TLASNode* child1 = &tlasNode[node->leftRight & 0xffff];
		TLASNode* child2 = &tlasNode[node->leftRight >> 16];

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
}


int main(){
    // Define sensor:
    Scalar resolution[2] = {640,640};
    // Scalar resolution[2] = {100,100};
    Scalar size[2] = {30,30};
    SimpleSensor<Scalar> sensor(resolution, size);

    // Define camera:
    SimpleCamera<Scalar> camera(30, sensor, true);
    camera.set_position(0,0,-15);

    // Define a simple light:
    PointLight<Scalar> light(1);

    // Load geometry:
    int N = 4;
    Geometry<Scalar>* geometries = new Geometry<Scalar>[N];
    for (int i = 0; i < N; i++) {
        geometries[i].read_obj("../suzanne.obj");//, "obj");
        geometries[i].construct_triangles();
        geometries[i].build_bvh();
        geometries[i].set_position(Vector3<Scalar>(0,2.5*i - 7,9));
    }
    for (int i = 0; i < N; i++){
        std::cout << geometries[i].bvh->bounds.bmin[0] << ", " << geometries[i].bvh->bounds.bmin[1] << ", " << geometries[i].bvh->bounds.bmin[2] << "\n";
        std::cout << geometries[i].bvh->bounds.bmax[0] << ", " << geometries[i].bvh->bounds.bmax[1] << ", " << geometries[i].bvh->bounds.bmax[2] << "\n";
    }

    // Create the scene:
    auto tlas = TLAS(geometries, N);

    // Build BVH:
    std::cout << "Building BVH...\n";
    auto start = std::chrono::high_resolution_clock::now();

    tlas.Build();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "    BVH built in " << duration.count() << " milliseconds\n";

    // Render (WITH TILES):
    int tile_size = 16;
    std::cout << "Rendering with " << sensor.get_resolution_h()*sensor.get_resolution_v() << " rays...\n";
    start = std::chrono::high_resolution_clock::now();
    uint8_t  image[(int) sensor.get_resolution_h()][(int) sensor.get_resolution_v()] = {0};
    for( int u = 0; u < sensor.get_resolution_h(); u+=tile_size) {
        for (int v = 0; v < sensor.get_resolution_v(); v+=tile_size){
            for (int x = 0; x < tile_size; x++){
                for (int y = 0; y < tile_size; y++){
                    auto ray = camera.pixel_to_ray(u+x,v+y);
                    // scene.intersect( ray );
                    tlas.Intersect( ray );

                    if (ray.hit.t < std::numeric_limits<Scalar>::max()) {
                        image[u+x][v+y] = 255;
                    }
                }
            }
        }
    }
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "    Rendered in " << duration.count() << " milliseconds\n";
    std::string filename = "frame_bvh.ppm";
    std::ofstream ppm2(filename, std::ios::binary);
    ppm2 << "P5\n" << sensor.get_resolution_h() << ' ' << sensor.get_resolution_v() << "\n255\n";
    ppm2.write(reinterpret_cast<char *>(image), sensor.get_resolution_h() * sensor.get_resolution_v() * sizeof(uint8_t));
    ppm2.flush();

    return 0;
}