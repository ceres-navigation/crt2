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



struct TLASNode {
    Vector3<Scalar> aabbMin;
    uint leftBLAS;
    Vector3<Scalar> aabbMax;
    uint isLeaf;
};

class TLAS {
    public:
        TLAS() = default;
        TLAS( BVH<Scalar>* bvhList, int N );
        void Build();
        void Intersect( Ray<Scalar>& ray );
    private:
        TLASNode* tlasNode = 0;
        BVH<Scalar>* blas = 0;
        uint nodesUsed, blasCount;
};

TLAS::TLAS( BVH<Scalar>* bvhList, int N ) {
    // copy a pointer to the array of bottom level accstructs
    blas = bvhList;
    blasCount = N;
    // allocate TLAS nodes
    tlasNode = new TLASNode[2*N];
    nodesUsed = 2;
}

void TLAS::Build() {
    // assign a TLASleaf node to each BLAS
    tlasNode[2].leftBLAS = 0;
    tlasNode[2].aabbMin = Vector3<Scalar>( -100 );
    tlasNode[2].aabbMax = Vector3<Scalar>( 100 );
    tlasNode[2].isLeaf = true;

    tlasNode[3].leftBLAS = 1;
    tlasNode[3].aabbMin = Vector3<Scalar>( -100 );
    tlasNode[3].aabbMax = Vector3<Scalar>( 100 );
    tlasNode[3].isLeaf = true;

    // create a root node over the two leaf nodes
    tlasNode[0].leftBLAS = 2;
    tlasNode[0].aabbMin = Vector3<Scalar>( -100 );
    tlasNode[0].aabbMax = Vector3<Scalar>( 100 );
    tlasNode[0].isLeaf = false;
}

void TLAS::Intersect( Ray<Scalar>& ray ) {
    TLASNode* node = &tlasNode[0], *stack[64];
    uint stackPtr = 0;
    while (1)
    {
        if (node->isLeaf)
        {
            blas[node->leftBLAS].Intersect( ray, 0);
            if (stackPtr == 0) break; else node = stack[--stackPtr];
            continue;
        }
        TLASNode* child1 = &tlasNode[node->leftBLAS];
        TLASNode* child2 = &tlasNode[node->leftBLAS + 1];
        float dist1 = intersect_aabb( ray, child1->aabbMin, child1->aabbMax );
        float dist2 = intersect_aabb( ray, child2->aabbMin, child2->aabbMax );
        if (dist1 > dist2) { std::swap( dist1, dist2 ); std::swap( child1, child2 ); }
        if (dist1 == std::numeric_limits<Scalar>::max())
        {
            if (stackPtr == 0) break; else node = stack[--stackPtr];
        }
        else
        {
            node = child1;
            if (dist2 != std::numeric_limits<Scalar>::max()) stack[stackPtr++] = child2;
        }
    }
}


int main(){
    // Define sensor:
    Scalar resolution[2] = {640,640};
    Scalar size[2] = {30,30};
    SimpleSensor<Scalar> sensor(resolution, size);

    // Define camera:
    SimpleCamera<Scalar> camera(30, sensor, true);
    camera.set_position(0,0,-8);

    // Define a simple light:
    PointLight<Scalar> light(1);

    // Load geometry:
    Geometry<Scalar>* geometries = new Geometry<Scalar>[4];
    geometries[0].read_obj("../suzanne.obj");
    geometries[0].set_position(Vector3<Scalar>(0,1,0));

    geometries[1].read_obj("../suzanne.obj");
    geometries[1].set_position(Vector3<Scalar>(0,-1,0));

    geometries[0].build_bvh();
    geometries[1].build_bvh();

    // Create the scene:
    // Scene<Scalar> scene(geometries, 4);
    // bvh[0] = BVH( "assets/armadillo.tri", 30000 );
    // bvh[1] = BVH( "assets/armadillo.tri", 30000 );
    BVH<Scalar>* bvhs = new BVH<Scalar>[2];
    bvhs[0] = *geometries[0].bvh;
    bvhs[1] = *geometries[1].bvh;
    auto tlas = TLAS( bvhs, 2 );

    // Build BVH:
    std::cout << "Building BVH...\n";
    auto start = std::chrono::high_resolution_clock::now();
    // scene.build_bvh();
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

                    // Format an output image:
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