#define TINYOBJLOADER_IMPLEMENTATION

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cassert>
#include <limits>
#include <cstring>
#include <memory>
#include <vector>
#include <random>

#include <zstd.h>
#include <tiny_obj_loader.h>

#include "primitives/geometry.hpp"
#include "acceleration/bvh.hpp"

#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"

template <typename Scalar>
Geometry<Scalar>::Geometry(){
};

template <typename Scalar>
Geometry<Scalar>::Geometry(const char* file_path, std::string file_type){
    if (file_type == "obj"){
        read_obj(file_path);
    }
    else if (file_type == "binary"){
        // read_binary(file_path);
        std::cout << "BINARY IS NOT YET IMPLEMENTED AND TEST!\n";
        exit(1);
    }
    else {
        std::cout << file_type << "is INVALID.  Must be 'obj' or 'binary'.\n";
        exit(2);
    }

    this->construct_triangles();

    this->build_bvh();
};

template <typename Scalar>
Geometry<Scalar>::~Geometry(){
    delete this->triangles;
    delete this->triangle_data;
    delete this->bvh;
};

template <typename Scalar>
void Geometry<Scalar>::intersect(Ray<Scalar> &ray){
    this->bvh->Intersect(ray);
}

template <typename Scalar>
void Geometry<Scalar>::set_scale(Scalar new_scale){
    this->scale = new_scale;

    this->bvh->scale = new_scale;
    this->bvh->transform_changed = true;
    this->bvh->UpdateBounds();
};
template <typename Scalar>
void Geometry<Scalar>::set_position(Vector3<Scalar> new_position) {
    this->position = new_position;

    this->bvh->position = new_position;
    this->bvh->transform_changed = true;
    this->bvh->transform_nodes = new TransformNode<Scalar>(new_position);
    this->bvh->num_transformations = 1;
    this->bvh->UpdateBounds();
};
template <typename Scalar>
void Geometry<Scalar>::set_rotation(Rotation<Scalar> new_rotation){
    this->rotation = new_rotation;

    this->bvh->rotation = new_rotation;
    this->bvh->transform_changed = true;
    this->bvh->UpdateBounds();
};
template <typename Scalar>
void Geometry<Scalar>::set_pose(Vector3<Scalar> new_position, Rotation<Scalar> new_rotation){
    this->position = new_position;
    this->rotation = new_rotation;

    this->bvh->position = new_position;
    this->bvh->rotation = new_rotation;
    this->bvh->transform_changed = true;
    this->bvh->UpdateBounds();
};
template <typename Scalar>
void Geometry<Scalar>::set_transform(Scalar new_scale, Vector3<Scalar> new_position, Rotation<Scalar> new_rotation){
    this->scale    = new_scale;
    this->position = new_position;
    this->rotation = new_rotation;

    this->bvh->scale    = new_scale;
    this->bvh->position = new_position;
    this->bvh->rotation = new_rotation;
    this->bvh->transform_changed = true;
    this->bvh->UpdateBounds();
};

template <typename Scalar>
void Geometry<Scalar>::build_bvh(int BINS){
    this->bvh = new BVH<Scalar>(this->triangles, this->num_triangles);
    this->bvh->Build(BINS);
}

template <typename Scalar>
void Geometry<Scalar>::construct_triangles(){
    // Get the number of triangles:
    this->num_triangles = this->faces.size();

    // Allocate triangles on heap:
    this->triangles = new Triangle<Scalar>[this->num_triangles];

    // Populate triangles with the data read in from a file:
    for (uint32_t i = 0; i < num_triangles; i++){
        auto face_def = this->faces[i];

        auto v0 = Vector3<Scalar>(this->vertices[face_def[0]][0], this->vertices[face_def[0]][1], this->vertices[face_def[0]][2]);
        auto v1 = Vector3<Scalar>(this->vertices[face_def[1]][0], this->vertices[face_def[1]][1], this->vertices[face_def[1]][2]);
        auto v2 = Vector3<Scalar>(this->vertices[face_def[2]][0], this->vertices[face_def[2]][1], this->vertices[face_def[2]][2]);

        this->triangles[i].vertex0 = v0;
        this->triangles[i].vertex1 = v1;
        this->triangles[i].vertex2 = v2;

        // TODO: Include additional triangle data...
    }
};

template <typename Scalar>
void Geometry<Scalar>::read_obj(const char* file_path){
    this->vertices.clear();
    this->faces.clear();

    tinyobj::ObjReader reader;
    tinyobj::ObjReaderConfig reader_config;

    if (!reader.ParseFromFile(file_path, reader_config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
        }
        exit(1);
    }

    if (!reader.Warning().empty()) {
    std::cout << "TinyObjReader: " << reader.Warning();
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();

    // Loop over vertices:
    for (size_t i = 0; i < attrib.vertices.size(); i=i+3){
        std::vector<Scalar> vertex;
        tinyobj::real_t vx_i = attrib.vertices[size_t(i)+0];
        tinyobj::real_t vy_i = attrib.vertices[size_t(i)+1];
        tinyobj::real_t vz_i = attrib.vertices[size_t(i)+2];
        vertex.push_back((Scalar) vx_i);
        vertex.push_back((Scalar) vy_i);
        vertex.push_back((Scalar) vz_i);
        this->vertices.push_back(vertex);
    }

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

            // Loop over vertices in the face.
            std::vector<uint32_t> face_def;
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx_i = shapes[s].mesh.indices[index_offset + v];
                face_def.push_back((uint32_t) idx_i.vertex_index);
            }
            this->faces.push_back(face_def);
            index_offset += fv;

            // per-face material
            shapes[s].mesh.material_ids[f];
        }
    }
};

template <typename Scalar>
void Geometry<Scalar>::read_binary(const char* file_path){
    this->vertices.clear();
    this->faces.clear();

    char magic_return[magic_length];
    uint32_t compressed_v_size;
    uint32_t compressed_f_size;
    uint32_t num_v;
    uint32_t num_f;

    // Read the file:
    FILE* file = fopen(file_path, "rb");

    // Verify magic keyword:
    size_t fread_ret;
    fread_ret = fread(&magic_return, sizeof(char), 6, file);
    if (fread_ret){ std::cout << fread_ret <<"\n";};
    fread_ret = fread(&compressed_v_size, sizeof(uint32_t), 1, file);
    if (fread_ret){ std::cout << fread_ret <<"\n";};
    fread_ret = fread(&compressed_f_size, sizeof(uint32_t), 1, file);
    if (fread_ret){ std::cout << fread_ret <<"\n";};
    fread_ret = fread(&num_v, sizeof(uint32_t), 1, file);
    if (fread_ret){ std::cout << fread_ret <<"\n";};
    fread_ret = fread(&num_f, sizeof(uint32_t), 1, file);
    if (fread_ret){ std::cout << fread_ret <<"\n";};

    // TODO: DEBUG WHY THIS DOESNT WORK:
    for (int i = 0; i < magic_length; i++){
        assert(magic_return[i] == magic[i]);
    }

    // Decompress vertices:
    auto v_compressed = new uint8_t[compressed_v_size];
    fread_ret = fread(v_compressed, sizeof(uint8_t), compressed_v_size, file);
    if (fread_ret){ std::cout << fread_ret <<"\n";};
    auto v_array = new Scalar[num_v][3];
    auto size_v = ZSTD_decompress(v_array, num_v*3*sizeof(Scalar), v_compressed, compressed_v_size);
    if (ZSTD_isError(size_v)) {
        std::cerr << compressed_v_size << " " << ZSTD_getErrorName(size_v) << std::endl;
    }

    // Decompress faces:
    auto f_compressed = new uint8_t[compressed_f_size];
    fread_ret = fread(f_compressed, sizeof(uint8_t), compressed_f_size, file);
    if (fread_ret){ std::cout << fread_ret <<"\n";};
    auto f_array = new uint32_t[num_f][3];
    auto size_f = ZSTD_decompress(f_array, num_f*3*sizeof(uint32_t), f_compressed, compressed_f_size);
    if (ZSTD_isError(size_f)) {
        std::cerr << compressed_f_size << " " << ZSTD_getErrorName(size_f) << std::endl;
    }

    fclose(file);

    // Convert 2d array to vector of vector:
    for (uint32_t i = 0; i < num_v; i++){
        std::vector<Scalar> vertex;
        for (int j = 0; j < 3; j++){
            vertex.push_back(v_array[i][j]);
        }
        this->vertices.push_back(vertex);
    }

    // Convert 2d array to vector of vector:
    for (uint32_t i = 0; i < num_f; i++){
        std::vector<uint32_t> face_def;
        for (int j = 0; j < 3; j++){
            face_def.push_back(f_array[i][j]);
        }
        this->faces.push_back(face_def);
    }

    // Release heap allocated arrays:
    delete [] v_compressed;
    delete [] v_array;
    delete [] f_compressed;
    delete [] f_array;

    // Get the triangles:
    this->construct_triangles();
};

template <typename Scalar>
void Geometry<Scalar>::write_binary(const char* file_path){
    uint32_t num_v = this->vertices.size();
    uint32_t num_f = this->faces.size();

    auto v_array = new Scalar[num_v][3];
    auto f_array = new uint32_t[num_f][3];

    // Convert vector of vector to 2d array:
    for (uint32_t i = 0; i < num_v; i++){
        for (int j = 0; j < 3; j++){
            v_array[i][j] = this->vertices[i][j];
        }
    }

    // Convert vector of vector to 2d array:
    for (uint32_t i = 0; i < num_f; i++){
        for (int j = 0; j < 3; j++){
            f_array[i][j] = this->faces[i][j];
        }
    }

    size_t size_bound;

    // Compress the vertices:
    size_t v_size = num_v*3*sizeof(Scalar);
    size_bound = ZSTD_compressBound(v_size);
    auto compressed_v = new Scalar[size_bound];
    uint32_t compressed_v_size = ZSTD_compress(compressed_v, size_bound, v_array, v_size, 4);
    if (ZSTD_isError(compressed_v_size)) {
        std::cerr << ZSTD_getErrorName(compressed_v_size) << std::endl;
        assert(!ZSTD_isError(compressed_v_size));
    }

    // Compress the faces:
    size_t f_size = num_f*3*sizeof(uint32_t);
    size_bound = ZSTD_compressBound(f_size);
    auto compressed_f = new uint8_t[size_bound];
    uint32_t compressed_f_size = ZSTD_compress(compressed_f, size_bound, f_array, f_size, 4);
    if (ZSTD_isError(compressed_f_size)) {
        std::cerr << ZSTD_getErrorName(compressed_f_size) << std::endl;
        assert(!ZSTD_isError(compressed_f_size));
    }

    // Write uncompressed header to file:
    FILE* file = fopen(file_path, "wb");
    fwrite(magic.c_str(), sizeof(char), magic_length, file);
    fwrite(&compressed_v_size, sizeof(uint32_t), 1, file);
    fwrite(&compressed_f_size, sizeof(uint32_t), 1, file);
    fwrite(&num_v, sizeof(uint32_t), 1, file);
    fwrite(&num_f, sizeof(uint32_t), 1, file);

    // Write out the compressed data blocks:
    fwrite(compressed_v, compressed_v_size, 1, file);
    fwrite(compressed_f, compressed_f_size, 1, file);
    fclose(file);

    // Release heap allocated arrays:
    delete [] v_array;
    delete [] f_array;
    delete [] compressed_v;
    delete [] compressed_f;
};



// Explicitly Instantiate floats and doubles:
template class Geometry<float>;
template class Geometry<double>;