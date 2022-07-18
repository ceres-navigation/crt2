#ifndef __SCENE_H_
#define __SCENE_H_

#include <vector>

#include "cameras/camera.hpp"
#include "lights/light.hpp"
#include "geometry.hpp"

template <typename Scalar>
struct TLASNode {
    Vector3<Scalar> aabbMin;
    uint leftRight; // 2x16 bits
    Vector3<Scalar> aabbMax;
    uint blas_id;
    bool isLeaf() { return leftRight == 0; }
};

template <typename Scalar>
class Scene {
    public:
        Scene( std::vector<Geometry<Scalar>*> geometry_list);
        void Build();
        void Intersect( Ray<Scalar>& ray, uint tile_number);
        void unload(uint tile_number, uint max_missed_tiles);

        std::vector<uint8_t> render(Camera<Scalar>& camera, std::vector<Light<Scalar>*> lights);
        
    private:
        std::vector<Geometry<Scalar>*> geometry_list;
        uint FindBestMatch( uint* list, uint N, uint A );
        TLASNode<Scalar>* tlasNode = 0;
        BVH<Scalar>** blas = 0;
        uint nodesUsed, blasCount;
};

#endif