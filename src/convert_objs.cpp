#include <iostream>
#include <filesystem>
#include <chrono>
#include <stdio.h>
#include <string.h>

#include <tbb/parallel_for.h>

#include "geometry.hpp"

int parse_arguments(int argc, char** argv, std::string &input, std::string &output, 
                    bool &use_doubles, bool &use_parallel, bool &is_file, bool &print_help){
    if (argc == 1){
        std::cout << "Must provide a source .OBJ or directory containing multiple .OBJ files\n";
        return 1;
    }

    // Assign default values:
    output = ".";
    use_doubles = false;
    use_parallel = false;
    is_file = false;
    print_help = false;
    bool output_provided = false;
    bool output_is_file = false;

    // Verify an input was provided:
    // Check if help is requested:
    if (strcmp(argv[1],"-h")==0){
        print_help = true;
        return 0;
    }
    input = argv[1];

    // Check if input is a single file:
    const std::filesystem::path path(input);
    std::error_code ec;
    if (std::filesystem::is_regular_file(path, ec)){
        is_file = true;
    }
    if (ec) // Optional handling of possible errors. Usage of the same ec object works since fs functions are calling ec.clear() if no errors occur.
    {
        std::cerr << "Error in is_regular_file: " << ec.message();
    }

    for (int i = 2; i < argc; i++) {
        // Parse doubles argument:
        if (strcmp(argv[i],"-d")==0) {
            use_doubles = true; //TODO THIS CURRENTLY DOES NOTHING BECAUSE NO CODE IS TEMPLATED
        };

        // Parse parallel argument:
        if (strcmp(argv[i],"-p")==0){
            use_parallel = true;
        }

        // Check if help is requested:
        if (strcmp(argv[i],"-h")==0){
            print_help = true;
            return 0;
        }

        // Parse output:
        if (strcmp(argv[i],"-o")==0){
            if (i == argc-1){
                std::cout << "-o specified but no output directory/file provided\n";
                return 1;
            }
            output = argv[i+1];
            output_provided = true;

            // Check if input is a single file:
            const std::filesystem::path path(output);
            std::error_code ec;
            if (std::filesystem::is_regular_file(path, ec)){
                output_is_file = true;
            }
            if (ec) // Optional handling of possible errors. Usage of the same ec object works since fs functions are calling ec.clear() if no errors occur.
            {
                std::cerr << "Error in is_regular_file: " << ec.message();
            }
        }
    }

    if (!is_file && output_is_file){
        std::cout << "Cannot specify an input directory with output file";
        return 1;
    }

    if (is_file && !output_provided){
        // Create the file name for the binary file:
        const std::filesystem::path p(input);
        auto filename = p.stem().string();
        filename = filename.substr(0,filename.find_last_of('.')) + ".bin";
        std::filesystem::path output_filename = output.c_str();
        output_filename /= filename;
        output = output_filename;
    }
    else if (is_file && output_provided && !output_is_file){
        // Create the file name for the binary file:
        const std::filesystem::path p(input);
        auto filename = p.stem().string();
        filename = filename.substr(0,filename.find_last_of('.')) + ".bin";
        std::filesystem::path output_filename = output.c_str();
        output_filename /= filename;
        output = output_filename;
    }

    return 0;
};

int main(int argc, char** argv)
{  
    std::string input;
    std::string output;
    bool use_doubles;
    bool use_parallel;
    bool is_file;
    bool print_help;
    int ret = parse_arguments(argc, argv, input, output, use_doubles, use_parallel, is_file, print_help);

    if (ret != 0){
        return ret;
    }

    if (print_help){
        std::cout << "Usage\n";
        std::cout << "  convert_objs <path-to-inputs> [options]\n\n";
        std::cout << "Speciofy a source .OBJ or directory containing multiple .OBJ files\n";
        std::cout << "to generate compressed binary geometries.\n\n";
        std::cout << "Options\n";
        std::cout << "  -o <path-to-output>  = Output path (defaults to \".\")\n";
        std::cout << "  -p                   = Use parallel (via TBB)\n";
        std::cout << "  -d                   = Use double precision\n";
        std::cout << "  -h                   = Display this help message\n";
        return 0;
    }

    std::string extension(".obj");

    if (is_file){
        auto start = std::chrono::system_clock::now();
        if (use_doubles){
            Geometry<double> geometry(input.c_str(),"obj");
            geometry.write_binary(output.c_str());
        }
        else{
            Geometry<float> geometry(input.c_str(),"obj");
            geometry.write_binary(output.c_str());
        }
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        std::cout << input << " compressed and written to " << output << " in " << elapsed_seconds.count() << " seconds\n";
    }
    else {
        if (use_parallel){
            // Create a vector of all the files:
            std::vector<std::filesystem::path> input_files;
            for (auto &p : std::filesystem::recursive_directory_iterator(input.c_str())) {
                if (p.path().extension() == extension){
                    input_files.push_back(p.path());
                }
            }

            tbb::parallel_for(tbb::blocked_range<int>(0, input_files.size()), [&](tbb::blocked_range<int> r){
                for (int i = r.begin(); i<r.end(); ++i){
                    auto start = std::chrono::system_clock::now();
                    auto path = input_files[i];

                    // Create the file name for the binary file:
                    auto filename = path.stem().string();
                    filename = filename.substr(0,filename.find_last_of('.')) + ".bin";
                    std::filesystem::path output_filename = output.c_str();
                    output_filename /= filename;

                    if (use_doubles){
                        Geometry<double> geometry(path.c_str(),"obj");
                        geometry.write_binary(output_filename.c_str());
                    }
                    else {
                        Geometry<float> geometry(path.c_str(),"obj");
                        geometry.write_binary(output_filename.c_str());
                    }

                    auto end = std::chrono::system_clock::now();
                    std::chrono::duration<double> elapsed_seconds = end - start;
                    std::cout << path.c_str() << " compressed and written to " << output_filename.c_str() << " in " << elapsed_seconds.count() << " seconds\n";
                }
            });
        }
        else {
            for (auto &p : std::filesystem::recursive_directory_iterator(input.c_str())) {
                if (p.path().extension() == extension){
                    auto start = std::chrono::system_clock::now();

                    // Create the file name for the binary file:
                    auto filename = p.path().stem().string();
                    filename = filename.substr(0,filename.find_last_of('.')) + ".bin";
                    std::filesystem::path output_filename = output.c_str();
                    output_filename /= filename;

                    if (use_doubles){
                        Geometry<double> geometry(p.path().c_str(),"obj");
                        geometry.write_binary(output_filename.c_str());
                    }
                    else {
                        Geometry<float> geometry(p.path().c_str(),"obj");
                        geometry.write_binary(output_filename.c_str());
                    }


                    auto end = std::chrono::system_clock::now();
                    std::chrono::duration<double> elapsed_seconds = end - start;
                    std::cout << p.path().c_str() << " compressed and written to " << output_filename.c_str() << " in " << elapsed_seconds.count() << " seconds\n";
                }
            }
        }
    }

    return 0;
}