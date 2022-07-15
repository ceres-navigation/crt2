#ifndef __BINARY_FILE_H_
#define __BINARY_FILE_H_

#include "primitives/aabb.hpp"

#include <fstream>
#include <zstd.h>

template <typename Scalar>
bool using_doubles();

template <typename Scalar>
void read_bounds(FILE* file, bool is_double, AABB<Scalar> &bounds);

template <typename Scalar>
void read_data(FILE* file, bool is_double, uint32_t t_compressed_size, uint32_t num_triangles, Scalar t_array[][9]);


#endif