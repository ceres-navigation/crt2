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

#include "geometry.hpp"
#include "acceleration/bvh.hpp"

#include "primitives/triangle.hpp"
#include "primitives/ray.hpp"
#include "primitives/aabb.hpp"

#include "materials/material.hpp"
#include "materials/lambertian.hpp"

template <typename Scalar>
bool using_doubles();

template <>
bool using_doubles<float>() {return false;};

template <>
bool using_doubles<double>() {return true;};

template <typename Scalar>
void read_file(FILE* file, bool is_double, AABB<Scalar> &bounds, uint32_t t_compressed_size, uint32_t num_triangles, Scalar t_array[][9]);

template <>
void read_file<float>(FILE* file, bool is_double, AABB<float> &bounds, uint32_t t_compressed_size, uint32_t num_triangles, float t_array[][9]){
    size_t fread_ret;
    auto t_compressed = new uint8_t[t_compressed_size];
    if (is_double){
        // Read the bounding box:
        double val;
        for (int i = 0; i < 3; i++){
            fread_ret = fread(&val, sizeof(double),  1, file);
            if (fread_ret != 1){std::cout<<"failed reading bounds.bmin["<<i<<"]\n";exit(2);};
            bounds.bmin[i] = (float) val;

            fread_ret = fread(&val, sizeof(double),  1, file);
            if (fread_ret != 1){std::cout<<"failed reading bounds.bmax["<<i<<"]\n";exit(2);};
            bounds.bmax[i] = (float) val;
        }

        // Decompress triangles:
        fread_ret = fread(t_compressed, sizeof(uint8_t), t_compressed_size, file);
        // if (fread_ret){ std::cout << fread_ret <<"\n";};
        auto t_array2 = new double[num_triangles][9];
        auto t_size = ZSTD_decompress(t_array2, num_triangles*9*sizeof(double), t_compressed, t_compressed_size);
        if (ZSTD_isError(t_size)) {
            std::cerr << t_compressed_size << " " << ZSTD_getErrorName(t_size) << std::endl;
        }
        for (uint32_t i = 0; i < num_triangles; i++){
            for (int j = 0; j < 9; j++){
                t_array[i][j] = (float) t_array2[i][j];
            }
        }
        delete [] t_array2;
    } else {
        // Read the bounding box:
        for (int i = 0; i < 3; i++){
            fread_ret = fread(&bounds.bmin[i], sizeof(float),  1, file);
            if (fread_ret != 1){std::cout<<"failed reading bounds.bmin["<<i<<"]\n";exit(2);};

            fread_ret = fread(&bounds.bmax[i], sizeof(float),  1, file);
            if (fread_ret != 1){std::cout<<"failed reading bounds.bmax["<<i<<"]\n";exit(2);};
        }

        // Decompress triangles:
        fread_ret = fread(t_compressed, sizeof(uint8_t), t_compressed_size, file);
        // if (fread_ret){ std::cout << fread_ret <<"\n";};
        auto t_size = ZSTD_decompress(t_array, num_triangles*9*sizeof(float), t_compressed, t_compressed_size);
        if (ZSTD_isError(t_size)) {
            std::cerr << t_compressed_size << " " << ZSTD_getErrorName(t_size) << std::endl;
        }
    }
    delete [] t_compressed;
}

template <>
void read_file<double>(FILE* file, bool is_double, AABB<double> &bounds, uint32_t t_compressed_size, uint32_t num_triangles, double t_array[][9]){
    size_t fread_ret;
    auto t_compressed = new uint8_t[t_compressed_size];
    if (is_double){
        // Read the bounding box:
        for (int i = 0; i < 3; i++){
            fread_ret = fread(&bounds.bmin[i], sizeof(double), 1, file);
            if (fread_ret != 1){std::cout<<"failed reading bounds.bmin["<<i<<"]\n";exit(2);};

            fread_ret = fread(&bounds.bmax[i], sizeof(double), 1, file);
            if (fread_ret != 1){std::cout<<"failed reading bounds.bmax["<<i<<"]\n";exit(2);};
        }

        // Decompress triangles:
        fread_ret = fread(t_compressed, sizeof(uint8_t), t_compressed_size, file);
        // if (fread_ret){ std::cout << fread_ret <<"\n";};
        auto t_size = ZSTD_decompress(t_array, num_triangles*9*sizeof(double), t_compressed, t_compressed_size);
        if (ZSTD_isError(t_size)) {
            std::cerr << t_compressed_size << " " << ZSTD_getErrorName(t_size) << std::endl;
        }
    } else {
        // Read the bounding box:
        float val;
        for (int i = 0; i < 3; i++){
            fread_ret = fread(&val, sizeof(float),  1, file);
            if (fread_ret != 1){std::cout<<"failed reading bounds.bmin["<<i<<"]\n";exit(2);};
            bounds.bmin[i] = (float) val;

            fread_ret = fread(&val, sizeof(float),  1, file);
            if (fread_ret != 1){std::cout<<"failed reading bounds.bmax["<<i<<"]\n";exit(2);};
            bounds.bmax[i] = (float) val;
        }

        // Decompress triangles:
        fread_ret = fread(t_compressed, sizeof(uint8_t), t_compressed_size, file);
        // if (fread_ret){ std::cout << fread_ret <<"\n";};
        auto t_array2 = new float[num_triangles][9];
        auto t_size = ZSTD_decompress(t_array2, num_triangles*9*sizeof(float), t_compressed, t_compressed_size);
        if (ZSTD_isError(t_size)) {
            std::cerr << t_compressed_size << " " << ZSTD_getErrorName(t_size) << std::endl;
        }
        for (uint32_t i = 0; i < num_triangles; i++){
            for (int j = 0; j < 9; j++){
                t_array[i][j] = (double) t_array2[i][j];
            }
        }
        delete [] t_array2;
    }
    delete [] t_compressed;
}

template <typename Scalar>
Geometry<Scalar>::Geometry(const char* file_path, std::string file_type){
    if (file_type == "obj"){
        read_obj(file_path);
    }
    else if (file_type == "binary"){
        read_binary(file_path);
    }
    else {
        std::cout << "file_type of: " << file_type << " is INVALID.  Must be 'obj' or 'binary'.\n";
        exit(2);
    }

    this->build_bvh();

    this->bvh->UpdateBounds();

    // Assign the material:
    this->material = new Lambertian<Scalar>();
};

template <typename Scalar>
Geometry<Scalar>::~Geometry(){
    delete this->triangles;
    delete this->triangle_data;
    delete this->bvh;
    // delete this->material; // ??
};

template <typename Scalar>
void Geometry<Scalar>::intersect(Ray<Scalar> &ray){
    this->bvh->Intersect(ray);
}

template <typename Scalar>
void Geometry<Scalar>::set_scale(Scalar new_scale){
    this->scale = new_scale;

    this->bvh->scale = new_scale;
    this->bvh->recip_scale = 1/new_scale;

    this->bvh->transform_changed = true;
    this->bvh->UpdateBounds();
};
template <typename Scalar>
void Geometry<Scalar>::set_position(Vector3<Scalar> new_position) {
    this->position = new_position;
    
    this->bvh->position = new_position;

    this->bvh->transform_changed = true;
    this->bvh->UpdateBounds();
};
template <typename Scalar>
void Geometry<Scalar>::set_rotation(Rotation<Scalar> new_rotation){
    this->rotation = new_rotation;

    this->bvh->rotation = new_rotation;
    this->bvh->inverse_rotation = new_rotation.transpose();

    this->bvh->transform_changed = true;
    this->bvh->UpdateBounds();
};
template <typename Scalar>
void Geometry<Scalar>::set_pose(Vector3<Scalar> new_position, Rotation<Scalar> new_rotation){
    this->position = new_position;
    this->rotation = new_rotation;

    this->bvh->position = new_position;
    this->bvh->rotation = new_rotation;
    this->bvh->inverse_rotation = new_rotation.transpose();

    this->bvh->transform_changed = true;
    this->bvh->UpdateBounds();
};
template <typename Scalar>
void Geometry<Scalar>::set_transform(Scalar new_scale, Vector3<Scalar> new_position, Rotation<Scalar> new_rotation){
    this->scale    = new_scale;
    this->position = new_position;
    this->rotation = new_rotation;

    this->bvh->scale    = new_scale;
    this->bvh->recip_scale = 1/new_scale;
    this->bvh->position = new_position;
    this->bvh->rotation = new_rotation;
    this->bvh->inverse_rotation = new_rotation.transpose();

    this->bvh->transform_changed = true;
    this->bvh->UpdateBounds();
};

template <typename Scalar>
void Geometry<Scalar>::build_bvh(int BINS){
    this->bvh = new BVH<Scalar>(this->triangles, this->num_triangles);
    this->bvh->set_parent(this);
    this->bvh->Build(BINS);
}

template <typename Scalar>
void Geometry<Scalar>::construct_triangles(std::vector<Vector3<Scalar>> vertices, 
                                           std::vector<std::vector<uint32_t>> faces,
                                           std::vector<Vector3<Scalar>> normals,
                                           std::vector<Vector2<Scalar>> texture_coordinates){
    // Get the number of triangles:
    this->num_triangles = faces.size();

    // Allocate triangles on heap:
    this->triangles = new Triangle<Scalar>[this->num_triangles];
    this->triangle_data = new TriangleData<Scalar>[this->num_triangles];

    // Populate triangles with the data read in from a file:
    for (uint32_t i = 0; i < num_triangles; i++){
        auto face_def = faces[i];

        // Calculate triangle vertices:
        this->triangles[i].vertex0 = vertices[face_def[0]];
        this->triangles[i].vertex1 = vertices[face_def[1]];
        this->triangles[i].vertex2 = vertices[face_def[2]];

        // this->triangle_data[i].vn0 = normals[face_def[0]];
        // this->triangle_data[i].vn1 = normals[face_def[1]];
        // this->triangle_data[i].vn2 = normals[face_def[2]];

        // Calculate face normal:
        Vector3<Scalar> edge1 = this->triangles[i].vertex1 - this->triangles[i].vertex0;
        Vector3<Scalar> edge2 = this->triangles[i].vertex2 - this->triangles[i].vertex0;
        this->triangle_data[i].face_normal = normalize<Scalar>(cross<Scalar>(edge1, edge2));
    }
};

template <typename Scalar>
void Geometry<Scalar>::read_obj(const char* file_path){
    std::vector<Vector3<Scalar>> vertices;
    std::vector<std::vector<uint32_t>> faces;
    std::vector<Vector3<Scalar>> normals;
    std::vector<Vector2<Scalar>> texture_coordinates;

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
        vertices.push_back(Vector3<Scalar>((Scalar) vx_i, (Scalar) vy_i, (Scalar) vz_i));
    }

    // Loop over vertex normals:
    for (size_t i = 0; i < attrib.normals.size(); i=i+3){

    }

    // Loop over texture coordinates:
    for (size_t i = 0; i < attrib.texcoords.size(); i=i+3){
        std::vector<Scalar> texcoord;
        tinyobj::real_t u = attrib.texcoords[size_t(i)+0];
        tinyobj::real_t v = attrib.texcoords[size_t(i)+1];
        texture_coordinates.push_back(Vector2<Scalar>((Scalar) u, (Scalar) v));
    }

    // Loop over shapes:
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
            faces.push_back(face_def);
            index_offset += fv;

            // per-face material
            shapes[s].mesh.material_ids[f];
        }
    }

    // Construct the triangles:
    this->construct_triangles(vertices, faces, normals, texture_coordinates);
};

template <typename Scalar>
void Geometry<Scalar>::read_binary(const char* file_path){
    std::vector<Vector3<Scalar>> vertices;
    std::vector<std::vector<uint32_t>> faces;
    std::vector<Vector3<Scalar>> normals;
    std::vector<Vector2<Scalar>> texture_coordinates;

    char magic_return[magic_length];
    uint32_t t_compressed_size;
    
    // Read the file:
    FILE* file = fopen(file_path, "rb");

    // Declare AABB:
    AABB<Scalar> bounds;

    // Determine if using doubles:
    bool is_double;

    // Verify magic keyword:
    size_t fread_ret;
    fread_ret = fread(&magic_return, sizeof(char), 6, file);
    if (fread_ret != 6){std::cout<<"failed reading magic phrase\n";  exit(2);};
    for (int i = 0; i < magic_length; i++){
        assert(magic_return[i] == magic[i]);
    }

    fread_ret = fread(&t_compressed_size, sizeof(uint32_t), 1, file);
    if (fread_ret != 1){std::cout<<"failed reading t_compressed\n";  exit(2);};
    fread_ret = fread(&num_triangles, sizeof(uint32_t), 1, file);
    if (fread_ret != 1){std::cout<<"failed reading num_triangles\n"; exit(2);};
    fread_ret = fread(&is_double, sizeof(bool), 1, file);
    if (fread_ret != 1){std::cout<<"failed reading is_double\n"; exit(2);};

    auto t_array = new Scalar[num_triangles][9];

    read_file<Scalar>(file, is_double, bounds, t_compressed_size, num_triangles, t_array);

    fclose(file);

    // Allocate triangles on heap:
    this->triangles = new Triangle<Scalar>[this->num_triangles];
    this->triangle_data = new TriangleData<Scalar>[this->num_triangles];

    // Convert array to triangles:
    for (uint32_t i = 0; i < num_triangles; i++){
        triangles[i].vertex0 = Vector3<Scalar>(t_array[i][0],t_array[i][1],t_array[i][2]);
        triangles[i].vertex1 = Vector3<Scalar>(t_array[i][3],t_array[i][4],t_array[i][5]);
        triangles[i].vertex2 = Vector3<Scalar>(t_array[i][6],t_array[i][7],t_array[i][8]);

        // Calculate face normal:
        Vector3<Scalar> edge1 = this->triangles[i].vertex1 - this->triangles[i].vertex0;
        Vector3<Scalar> edge2 = this->triangles[i].vertex2 - this->triangles[i].vertex0;
        this->triangle_data[i].face_normal = normalize<Scalar>(cross<Scalar>(edge1, edge2));
    }

    // Release heap allocated arrays:
    delete [] t_array;
};

template <typename Scalar>
void Geometry<Scalar>::write_binary(const char* file_path){
    // Convert vector of vector to 2d array:
    auto t_array = new Scalar[num_triangles][9];
    for (uint32_t i = 0; i < num_triangles; i++){
        t_array[i][0] = triangles[i].vertex0[0];
        t_array[i][1] = triangles[i].vertex0[1];
        t_array[i][2] = triangles[i].vertex0[2];
        t_array[i][3] = triangles[i].vertex1[0];
        t_array[i][4] = triangles[i].vertex1[1];
        t_array[i][5] = triangles[i].vertex1[2];
        t_array[i][6] = triangles[i].vertex2[0];
        t_array[i][7] = triangles[i].vertex2[1];
        t_array[i][8] = triangles[i].vertex2[2];
    }

    size_t size_bound;

    // Obtain the axis aligned bounding box:
    AABB<Scalar> bounds = this->bvh->bounds;

    // Compress the triangles:
    size_t t_size = num_triangles*9*sizeof(Scalar);
    size_bound = ZSTD_compressBound(t_size);
    auto t_compressed = new Scalar[size_bound];
    uint32_t t_compressed_size = ZSTD_compress(t_compressed, size_bound, t_array, t_size, 4);
    if (ZSTD_isError(t_compressed_size)) {
        std::cerr << ZSTD_getErrorName(t_compressed_size) << std::endl;
        assert(!ZSTD_isError(t_compressed_size));
    }

    bool is_double = using_doubles<Scalar>();

    // Write uncompressed header to file:
    FILE* file = fopen(file_path, "wb");
    fwrite(magic.c_str(), sizeof(char), magic_length, file);
    fwrite(&t_compressed_size, sizeof(uint32_t), 1, file);
    fwrite(&num_triangles, sizeof(uint32_t), 1, file);
    fwrite(&is_double, sizeof(bool), 1, file);
    for (int i = 0; i < 3; i++){
        fwrite(&bounds.bmin[i], sizeof(Scalar),  1, file);
        fwrite(&bounds.bmax[i], sizeof(Scalar),  1, file);
    }

    // Write out the compressed data blocks:
    fwrite(t_compressed, t_compressed_size, 1, file);
    fclose(file);

    // Release heap allocated arrays:
    delete [] t_array;
    delete [] t_compressed;
};



// Explicitly Instantiate floats and doubles:
template class Geometry<float>;
template class Geometry<double>;