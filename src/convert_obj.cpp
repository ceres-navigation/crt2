#include <iostream>

#include "geometry.hpp"

int main(int argc, char** argv)
{  
    if (argc != 3){
        std::cout << "Both a .OBJ and an output file must be specified\n";
        return 1;
    }

    Geometry geometry;

    geometry.read_obj(argv[1]);
    
    geometry.write_binary(argv[2]);

    return 0;
}