#include "binary_file.hpp"

#include "primitives/aabb.hpp"

#include <iostream>
#include <fstream>
#include <zstd.h>

template <>
bool using_doubles<float>() {return false;};

template <>
bool using_doubles<double>() {return true;};

template <>
void read_bounds<float>(FILE* file, bool is_double, AABB<float> &bounds){\
    size_t fread_ret;
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
    } else {
        // Read the bounding box:
        for (int i = 0; i < 3; i++){
            fread_ret = fread(&bounds.bmin[i], sizeof(float),  1, file);
            if (fread_ret != 1){std::cout<<"failed reading bounds.bmin["<<i<<"]\n";exit(2);};

            fread_ret = fread(&bounds.bmax[i], sizeof(float),  1, file);
            if (fread_ret != 1){std::cout<<"failed reading bounds.bmax["<<i<<"]\n";exit(2);};
        }
    }
};

template <>
void read_bounds<double>(FILE* file, bool is_double, AABB<double> &bounds){
    size_t fread_ret;
    if (is_double){
        // Read the bounding box:
        for (int i = 0; i < 3; i++){
            fread_ret = fread(&bounds.bmin[i], sizeof(double), 1, file);
            if (fread_ret != 1){std::cout<<"failed reading bounds.bmin["<<i<<"]\n";exit(2);};

            fread_ret = fread(&bounds.bmax[i], sizeof(double), 1, file);
            if (fread_ret != 1){std::cout<<"failed reading bounds.bmax["<<i<<"]\n";exit(2);};
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
    }
};

template <>
void read_data<float>(FILE* file, bool is_double, uint32_t t_compressed_size, uint32_t num_triangles, float t_array[][9]){
    size_t fread_ret;
    auto t_compressed = new uint8_t[t_compressed_size];
    if (is_double){
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
void read_data<double>(FILE* file, bool is_double, uint32_t t_compressed_size, uint32_t num_triangles, double t_array[][9]){
    size_t fread_ret;
    auto t_compressed = new uint8_t[t_compressed_size];
    if (is_double){
        // Decompress triangles:
        fread_ret = fread(t_compressed, sizeof(uint8_t), t_compressed_size, file);
        // if (fread_ret){ std::cout << fread_ret <<"\n";};
        auto t_size = ZSTD_decompress(t_array, num_triangles*9*sizeof(double), t_compressed, t_compressed_size);
        if (ZSTD_isError(t_size)) {
            std::cerr << t_compressed_size << " " << ZSTD_getErrorName(t_size) << std::endl;
        }
    } else {
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