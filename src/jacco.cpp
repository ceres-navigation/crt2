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

#include <chrono>
#include <vector>

// triangle count
#define N 64

using Scalar = float;

struct BVHNode {
    Vector3<Scalar> aabbMin, aabbMax;
    uint leftNode, firstTriIdx, triCount;
    bool isLeaf() { return triCount > 0; }
};

Triangle<Scalar> tri[N];
uint triIdx[N];
BVHNode bvhNode[N * 2 - 1];
uint rootNodeIdx = 0;
uint nodesUsed = 1;

void UpdateNodeBounds( uint nodeIdx )
{
    BVHNode& node = bvhNode[nodeIdx];
    node.aabbMin = Vector3<Scalar>( 1e30f );
    node.aabbMax = Vector3<Scalar>( -1e30f );
    for (uint first = node.firstTriIdx, i = 0; i < node.triCount; i++)
    {
        uint leafTriIdx = triIdx[first + i];
        Triangle<Scalar>& leafTri = tri[leafTriIdx];
        node.aabbMin = min( node.aabbMin, leafTri.vertex0 ),
        node.aabbMin = min( node.aabbMin, leafTri.vertex1 ),
        node.aabbMin = min( node.aabbMin, leafTri.vertex2 ),
        node.aabbMax = max( node.aabbMax, leafTri.vertex0 ),
        node.aabbMax = max( node.aabbMax, leafTri.vertex1 ),
        node.aabbMax = max( node.aabbMax, leafTri.vertex2 );
    }
}

void Subdivide( uint nodeIdx )
{
    // terminate recursion
    BVHNode& node = bvhNode[nodeIdx];
    if (node.triCount <= 2){
        return;
    } 

    // determine split axis and position
    Vector3<Scalar> extent = node.aabbMax - node.aabbMin;
    int axis = 0;
    if (extent[1] > extent[0]) axis = 1;
    if (extent[2] > extent[axis]) axis = 2;
    float splitPos = node.aabbMin[axis] + extent[axis] * (Scalar) 0.5f;

    // in-place partition
    int i = node.firstTriIdx;
    int j = i + node.triCount - 1;
    while (i <= j)
    {
        if (tri[triIdx[i]].centroid[axis] < splitPos)
            i++;
        else
            std::swap( triIdx[i], triIdx[j--] );
    }
    // abort split if one of the sides is empty
    uint leftCount = i - node.firstTriIdx;
    if (leftCount == 0 || leftCount == node.triCount){
        return;
    } 
    // create child nodes
    int leftChildIdx = nodesUsed++;
    int rightChildIdx = nodesUsed++;
    node.leftNode = leftChildIdx;
    bvhNode[leftChildIdx].firstTriIdx = node.firstTriIdx;
    bvhNode[leftChildIdx].triCount = leftCount;
    bvhNode[rightChildIdx].firstTriIdx = i;
    bvhNode[rightChildIdx].triCount = node.triCount - leftCount;
    node.triCount = 0;
    UpdateNodeBounds( leftChildIdx );
    UpdateNodeBounds( rightChildIdx );

    // recurse
    Subdivide( leftChildIdx );
    Subdivide( rightChildIdx );
}

void BuildBVH()
{
    for (int i = 0; i < N; i++){
        tri[i].centroid = (tri[i].vertex0 + tri[i].vertex1 + tri[i].vertex2) * (Scalar) 0.3333f;
    
        triIdx[i] = i;
    }

    // assign all triangles to root node
    BVHNode& root = bvhNode[rootNodeIdx];
    root.leftNode = 0;
    root.firstTriIdx = 0;
    root.triCount = N;
    UpdateNodeBounds( rootNodeIdx );

    // subdivide recursively
    Subdivide( rootNodeIdx );
}


void IntersectBVH( Ray<Scalar>& ray, const uint nodeIdx )
{
    BVHNode& node = bvhNode[nodeIdx];
    if (!intersect_aabb( ray, node.aabbMin, node.aabbMax )){
        return;
    }

    if (node.isLeaf()) {
        for (uint i = 0; i < node.triCount; i++ ){
            intersect_triangle( ray, tri[triIdx[node.firstTriIdx + i]] );
        }
    }
    else {
        IntersectBVH( ray, node.leftNode );
        IntersectBVH( ray, node.leftNode + 1 );
    }
}

int main(){
    // Define sensor:
    Scalar resolution[2] = {640,640};
    Scalar size[2] = {30,30};
    SimpleSensor<Scalar> sensor(resolution, size);

    // Define camera:
    SimpleCamera<Scalar> camera(30, sensor, true);
    camera.set_position(0,0,-15);

    // Define a simple light:
    PointLight<Scalar> light(1);

    // Load geometry and construct BVH:
    // Geometry<Scalar> geometry("cube");
    // // geometry.read_obj("../suzanne.obj");
    // geometry.read_obj("../cube.obj");
    // // geometry.read_obj("../random.obj");
    // Triangle<Scalar>* triangles = geometry.triangles;

    // Generate random triangles:
    std::default_random_engine generator;
    std::uniform_real_distribution<Scalar> distribution(0.0,1.0);
    std::cout << "Randomly generating " << N << " triangles...\n";
    for (int i = 0; i < N; i++)
    {
        Vector3<Scalar> r0( distribution(generator), distribution(generator), distribution(generator) );
        Vector3<Scalar> r1( distribution(generator), distribution(generator), distribution(generator) );
        Vector3<Scalar> r2( distribution(generator), distribution(generator), distribution(generator) );
        tri[i].vertex0 = r0 * (Scalar) 9 - Vector3<Scalar>( 5 );
        tri[i].vertex1 = tri[i].vertex0 + r1;
        tri[i].vertex2 = tri[i].vertex0 + r2;
    }

    // Ray trace:
    std::cout << "Beginning naive render...\n";
    auto start = std::chrono::high_resolution_clock::now();
    uint8_t  image[(int) sensor.get_resolution_h()][(int) sensor.get_resolution_v()] = {0};
    auto res_h = sensor.get_resolution_h();
    auto res_v = sensor.get_resolution_v();
    for( int u = 0; u < res_h; u++) {
        for (int v = 0; v < res_v; v++){
            auto ray = camera.pixel_to_ray(u,v);
            for (int i = 0; i < N; i++){
                intersect_triangle( ray, tri[i] );

                // Format an output image:
                if (ray.hit.t < std::numeric_limits<Scalar>::max()) {
                    image[u][v] = 255;
                }
            }
        }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "    Naive Render Duration: " << duration.count() << " milliseconds\n";
    std::string filename = "frame_naive.ppm";
    std::ofstream ppm(filename, std::ios::binary);
    ppm << "P5\n" << sensor.get_resolution_h() << ' ' << sensor.get_resolution_v() << "\n255\n";
    ppm.write(reinterpret_cast<char *>(image), sensor.get_resolution_h() * sensor.get_resolution_v() * sizeof(uint8_t));
    ppm.flush();

    // Build BVH:
    BuildBVH();
    std::cout << "Beginning bvh render...\n";
    start = std::chrono::high_resolution_clock::now();
    uint8_t  image2[(int) sensor.get_resolution_h()][(int) sensor.get_resolution_v()] = {0};
    for( int u = 0; u < res_h; u++) {
        for (int v = 0; v < res_v; v++){
            auto ray = camera.pixel_to_ray(u,v);
            IntersectBVH( ray, rootNodeIdx );

            // Format an output image:
            if (ray.hit.t < std::numeric_limits<Scalar>::max()) {
                image2[u][v] = 255;
            }
        }
    }
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "    BVH Render Duration: " << duration.count() << " milliseconds\n";
    filename = "frame_bvh.ppm";
    std::ofstream ppm2(filename, std::ios::binary);
    ppm2 << "P5\n" << sensor.get_resolution_h() << ' ' << sensor.get_resolution_v() << "\n255\n";
    ppm2.write(reinterpret_cast<char *>(image), sensor.get_resolution_h() * sensor.get_resolution_v() * sizeof(uint8_t));
    ppm2.flush();


    return 0;
}