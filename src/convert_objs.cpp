#include <iostream>

#include <filesystem>

#include "geometry.hpp"

int main(int argc, char** argv)
{  
    if (argc != 3){
        std::cout << "Both a directory of .OBJ files and an output directory must be specified\n";
        return 1;
    }

    std::string extension(".obj");

    for (auto &p : std::filesystem::recursive_directory_iterator(argv[1])) {
        if (p.path().extension() == extension){
            // Create geometry instance:
            Geometry geometry;

            // Read the current .OBJ file:
            geometry.read_obj(p.path().c_str());

            // Create the file name for the binary file:
            auto filename = p.path().stem().string();
            filename = filename.substr(0,filename.find_last_of('.')) + ".bin";
            std::filesystem::path output_filename = argv[2];
            output_filename /= filename;

            // Convert to compressed binary and write to file:
            geometry.write_binary(output_filename.c_str());
            std::cout << "\n";
        }
    }

    return 0;
}