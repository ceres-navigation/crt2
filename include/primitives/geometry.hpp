#ifndef __GEOMETRY_H_
#define __GEOMETRY_H_

#include <vector>
#include <stdint.h>

#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"

#include "utils/rigid_body.hpp"

#include "acceleration/bvh.hpp"

template <typename Scalar>
class Geometry : public RigidBody<Scalar>{
    public:
        // BINARY HEADER FORMAT:
        // {char magic[6], uint32_t compressed_v_size, uint32_t compressed_f_size, 
        //  uint32_t num_v, uint32_t num_f}
        // Define the magic phrase at start of the file:
        std::string magic = "CRTOBJ";
        int magic_length = 6;

        BVH<Scalar>* bvh = nullptr;

        // Definition of the geomtry to be stored:
        std::vector<std::vector<Scalar>> vertices; // TODO: REMOVE THIS!!!  Everything in triangle form
        std::vector<std::vector<uint32_t>> faces; // TODO: REMOVE THIS!!!  Everything in triangle form

        // Triangle defintions:
        uint32_t num_triangles = 0;
        Triangle<Scalar> *triangles = nullptr;
        TriangleData<Scalar> *triangle_data = nullptr;

        Geometry();

        Geometry(const char* file_path, std::string file_type);

        ~Geometry();

        void intersect(Ray<Scalar> &ray);

        void write_binary(const char* file_path);

        // Transform setting methods:
        void set_scale(Scalar new_scale);
        void set_position(Vector3<Scalar> new_position);
        void set_rotation(Rotation<Scalar> new_rotation);
        void set_pose(Vector3<Scalar> new_position, Rotation<Scalar> new_rotation);
        void set_transform(Scalar new_scale, Vector3<Scalar> new_position, Rotation<Scalar> new_rotation);

    // private: 
        void build_bvh(int BINS=8);

        void construct_triangles();

        // Read from file methods:
        void read_obj(const char* file_path);
        void read_binary(const char* file_path);
};

#endif