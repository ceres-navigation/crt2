#ifndef __GEOMETRY_H_
#define __GEOMETRY_H_

#include <vector>
#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"
#include <stdint.h>

#include "acceleration/bvh.hpp"

template <typename Scalar>
class Geometry {
    public:
        // BINARY HEADER FORMAT:
        // {char magic[6], uint32_t compressed_v_size, uint32_t compressed_f_size, 
        //  uint32_t num_v, uint32_t num_f}
        // Define the magic phrase at start of the file:
        const char* magic = "CRTOBJ";
        const int magic_length = 6;
        BVH<Scalar>* bvh;

        std::string name;

        // Definition of the geomtry to be stored:
        std::vector<std::vector<Scalar>> vertices; // TODO: REMOVE THIS!!!  Everything in triangle form
        std::vector<std::vector<uint32_t>> faces; // TODO: REMOVE THIS!!!  Everything in triangle form

        // Triangle defintions:
        uint32_t num_triangles = 0;
        Triangle<Scalar> *triangles;
        TriangleData<Scalar> *triangle_data;

        Geometry();

        Geometry(std::string name);

        ~Geometry();

        void read_obj(const char* file_path);

        void build_bvh(int BINS=8);
        void fast_build_bvh();

        void intersect(Ray<Scalar> &ray);

        void read_binary(const char* file_path);

        void write_binary(const char* file_path);

    private: 
        void construct_triangles();
};

#endif