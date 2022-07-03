#ifndef __GEOMETRY_H_
#define __GEOMETRY_H_

#include <vector>

using Scalar = double;

class Geometry {
    public:
        // BINARY HEADER FORMAT:
        // {char magic[6], uint32_t compressed_v_size, uint32_t compressed_f_size, 
        //  uint32_t num_v, uint32_t num_f}
        // Define the magic phrase at start of the file:
        const char* magic = "CRTOBJ";
        const int magic_length = 6;

        // Definition of the geomtry to be stored:
        std::vector<std::vector<Scalar>> vertices;
        std::vector<std::vector<uint32_t>> faces;

        Geometry();

        void read_obj(const char* file_path);

        void read_binary(const char* file_path);

        void write_binary(const char* file_path);
};

#endif