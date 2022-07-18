#ifndef __GEOMETRY_H_
#define __GEOMETRY_H_

#include <vector>
#include <stdint.h>

#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"

#include "utils/rigid_body.hpp"

#include "acceleration/bvh.hpp"

#include "materials/material.hpp"

// Forward declaration of Ray class:
template <typename Scalar>
class Ray; 

template <typename Scalar>
class Geometry : public RigidBody<Scalar>{
    public:
        // BINARY HEADER FORMAT:
        // {char magic[6], uint32_t compressed_v_size, uint32_t compressed_f_size, 
        //  uint32_t num_v, uint32_t num_f}
        // Define the magic phrase at start of the file:
        std::string magic = "CRTOBJ";
        int magic_length = 6;

        std::string binary_file_path;

        AABB<Scalar> bounds;
        BVH<Scalar>* bvh = nullptr;

        Material<Scalar>* material = nullptr;

        uint last_seen_tile_number;
        bool loaded = false;

        // Triangle defintions:
        uint32_t num_triangles = 0;
        Triangle<Scalar> *triangles = nullptr;
        TriangleData<Scalar> *triangle_data = nullptr;

        // Material definitions:
        bool smooth_shading = false; // TODO MAKE THIS CONFIGURABLE

        Geometry(const char* file_path, std::string file_type);

        ~Geometry();

        void intersect(Ray<Scalar> &ray, uint tile_number);

        void write_binary(const char* file_path);

        // Transform setting methods:
        void set_scale(Scalar new_scale);
        void set_position(Vector3<Scalar> new_position);
        void set_rotation(Rotation<Scalar> new_rotation);
        void set_pose(Vector3<Scalar> new_position, Rotation<Scalar> new_rotation);
        void set_transform(Scalar new_scale, Vector3<Scalar> new_position, Rotation<Scalar> new_rotation);


        // Lazy loading methods:
        void load(uint tile_number);
        void unload(uint tile_number, uint max_missed_tiles);

    private: 
        void build_bvh(int BINS=8);

        void bvh_aabb_only();

        void construct_triangles(std::vector<Vector3<Scalar>> vertices, 
                                 std::vector<std::vector<uint32_t>> faces,
                                 std::vector<Vector3<Scalar>> normals,
                                 std::vector<Vector2<Scalar>> texture_coordinates);

        // Read from file methods:
        void read_obj(const char* file_path);
        void read_binary(const char* file_path);
        void read_binary_header(const char* file_path);
};

#endif