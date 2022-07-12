#include "acceleration/scene.hpp"

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
}

// Explicitly Instantiate floats and doubles:
template class Scene<float>;
template class Scene<double>;