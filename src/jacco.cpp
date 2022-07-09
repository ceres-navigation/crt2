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

using Scalar = float;

template <typename Scalar>
struct BVHNode {
    Vector3<Scalar> aabbMin, aabbMax;
    uint leftFirst, triCount;
    bool isLeaf() { return triCount > 0; }
};

template <typename Scalar>
class BVH{
    public:
        Triangle<Scalar>* tri;
        uint N;
        uint nodesUsed;

        BVHNode<Scalar>* bvhNode;
        uint* triIdx;

        BVH(Triangle<Scalar>* triangles, uint num_triangles) {
            this->tri = triangles;
            this->N = num_triangles;
            this->bvhNode = new BVHNode<Scalar>[N*2];
            this->triIdx = new uint[N];

            this->nodesUsed = 1;

        };

        void UpdateNodeBounds( uint nodeIdx ) {
            BVHNode<Scalar>& node = bvhNode[nodeIdx];
            node.aabbMin = Vector3<Scalar>(  std::numeric_limits<Scalar>::max() );
            node.aabbMax = Vector3<Scalar>( -std::numeric_limits<Scalar>::max() );
            for (uint first = node.leftFirst, i = 0; i < node.triCount; i++)
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
        };

        void Subdivide( uint nodeIdx ) {
            // terminate recursion
            BVHNode<Scalar>& node = bvhNode[nodeIdx];
            if (node.triCount <= 2){
                return;
            } 

            // determine split axis and position
            Vector3<Scalar> extent = node.aabbMax - node.aabbMin;
            int axis = 0;
            if (extent[1] > extent[0]){
                axis = 1;
            }
            if (extent[2] > extent[axis]){
                axis = 2;
            }
            float splitPos = node.aabbMin[axis] + extent[axis] * (Scalar) 0.5f;

            // in-place partition
            int i = node.leftFirst;
            int j = i + node.triCount - 1;
            while (i <= j) {
                if (tri[triIdx[i]].centroid[axis] < splitPos) {
                    i++;
                }
                else {
                    std::swap( triIdx[i], triIdx[j--] );
                }
            }
            
            // abort split if one of the sides is empty
            uint leftCount = i - node.leftFirst;
            if (leftCount == 0 || leftCount == node.triCount){
                return;
            } 
            // create child nodes
            int leftChildIdx = nodesUsed++;
            int rightChildIdx = nodesUsed++;
            bvhNode[leftChildIdx].leftFirst = node.leftFirst;
            bvhNode[leftChildIdx].triCount = leftCount;
            bvhNode[rightChildIdx].leftFirst = i;
            bvhNode[rightChildIdx].triCount = node.triCount - leftCount;
            node.leftFirst = leftChildIdx;
            node.triCount = 0;
            UpdateNodeBounds( leftChildIdx );
            UpdateNodeBounds( rightChildIdx );

            // recurse
            Subdivide( leftChildIdx );
            Subdivide( rightChildIdx );
        };

        void Build() {
            for (int i = 0; i < N; i++){
                tri[i].centroid = (tri[i].vertex0 + tri[i].vertex1 + tri[i].vertex2) * (Scalar) 0.3333f;
            
                triIdx[i] = i;
            }

            // assign all triangles to root node
            BVHNode<Scalar>& root = bvhNode[0];
            root.leftFirst= 0;
            root.triCount = N;
            UpdateNodeBounds( 0 );

            // subdivide recursively
            Subdivide( 0 );
        };

        void Intersect( Ray<Scalar>& ray, const uint nodeIdx ) {
            BVHNode<Scalar>& node = bvhNode[nodeIdx];
            if (intersect_aabb( ray, node.aabbMin, node.aabbMax ) == std::numeric_limits<Scalar>::max()){
                return;
            }

            if (node.isLeaf()) {
                for (uint i = 0; i < node.triCount; i++ ){
                    intersect_triangle( ray, tri[triIdx[node.leftFirst + i]] );
                }
            }
            else {
                Intersect( ray, node.leftFirst );
                Intersect( ray, node.leftFirst + 1 );
            }
        };
};

int main(){
    // Define sensor:
    Scalar resolution[2] = {640,640};
    Scalar size[2] = {30,30};
    SimpleSensor<Scalar> sensor(resolution, size);

    // Define camera:
    SimpleCamera<Scalar> camera(30, sensor, true);
    camera.set_position(0,0,-5);

    // Define a simple light:
    PointLight<Scalar> light(1);

    // Load geometry and construct BVH:
    Geometry<Scalar> geometry("cube");
    geometry.read_obj("../suzanne.obj");
    // geometry.read_obj("../cube.obj");
    // // geometry.read_obj("../random.obj");
    // Triangle<Scalar>* triangles = geometry.triangles;

    // Build BVH:
    BVH<Scalar> bvh(geometry.triangles, geometry.num_triangles);
    bvh.Build();

    // Render:
    std::cout << "Beginning bvh render of " << geometry.num_triangles << " triangles...\n";
    auto start = std::chrono::high_resolution_clock::now();
    uint8_t  image[(int) sensor.get_resolution_h()][(int) sensor.get_resolution_v()] = {0};
    for( int u = 0; u < sensor.get_resolution_h(); u++) {
        for (int v = 0; v < sensor.get_resolution_v(); v++){
            auto ray = camera.pixel_to_ray(u,v);
            bvh.Intersect( ray, 0 );

            // Format an output image:
            if (ray.hit.t < std::numeric_limits<Scalar>::max()) {
                image[u][v] = 255;
            }
        }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "    BVH Render Duration: " << duration.count() << " milliseconds\n";
    std::string filename = "frame_bvh.ppm";
    std::ofstream ppm2(filename, std::ios::binary);
    ppm2 << "P5\n" << sensor.get_resolution_h() << ' ' << sensor.get_resolution_v() << "\n255\n";
    ppm2.write(reinterpret_cast<char *>(image), sensor.get_resolution_h() * sensor.get_resolution_v() * sizeof(uint8_t));
    ppm2.flush();


    return 0;
}