#ifndef __SCENE_H_
#define __SCENE_H_

#include "primitives/geometry.hpp"

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
        Scene() = default;
        Scene( Geometry<Scalar>* geometryList, uint N );
        void Build();
        void Intersect( Ray<Scalar>& ray );
        
    private:
        uint FindBestMatch( uint* list, uint N, uint A );
        TLASNode<Scalar>* tlasNode = 0;
        BVH<Scalar>* blas = 0;
        uint nodesUsed, blasCount;
};

#endif