#ifndef __E57_UTILS_H__
#define __E57_UTILS_H__

/*
************************ Last revision of this file ***********************
* $Author:: Michele Adduci
* $LastChangedDate:: 25/05/2019
**************************************************************************
*/

#include <cassert>
#include <iostream>
#include <string>
#include <utility>
#include <stdexcept>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

constexpr int CMD_SPACE{5};
constexpr int PARMS_SPACE{30};

constexpr auto print_helper = [](std::string cmd, std::string param, std::string description) {
    std::cout << std::right << setw(CMD_SPACE) << cmd
              << std::left << setw(PARMS_SPACE) << param << '\n'
              << std::left << setw(CMD_SPACE) << ""
              << "\t" << description << '\n'
              << '\n';
};

enum class FileType
{
    UNDEFINED,
    E57,
    PCD,
    PLY,
    OCT
};

[[nodiscard]] std::string file_type_to_string(FileType t)
{
    switch (t)
    {
    case FileType::E57:
        return "E57";
    case FileType::PCD:
        return "PCD";
    case FileType::OCT:
        return "OCT";
    case FileType::PLY:
        return "PLY";
    default:
        return "UNDEFINED";
    }
}

[[nodiscard]] bool is_valid_extension(fs::path &file_path, FileType ext)
{
    assert(ext == FileType::E57 || ext == FileType::PCD || ext == FileType::OCT || ext == FileType::PLY);
    if (file_path.string().back() != '/' && file_path.string().back() != '\\')
    {
        auto extension = file_path.extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(), ::toupper);
        if (extension == file_type_to_string(ext))
            return true;
    }
    return false;
}

[[nodiscard]] FileType get_file_type(fs::path &file_path)
{
    if (is_valid_extension(file_path, FileType::E57))
        return FileType::E57;
    else if (is_valid_extension(file_path, FileType::PCD))
        return FileType::PCD;
    else if (is_valid_extension(file_path, FileType::OCT))
        return FileType::OCT;
    else if (is_valid_extension(file_path, FileType::PLY))
        return FileType::PLY;
    else
        return FileType::UNDEFINED;
}

/**
 * Check the supported type of conversion between input and output
 */
[[nodiscard]] bool are_input_output_valid_match(FileType inputExt, FileType outputExt)
{
    return (inputExt == FileType::E57 && outputExt == FileType::PCD) ||
           (inputExt == FileType::PCD && outputExt == FileType::E57);
}

[[noreturn]] void print_help()
{
    std::cout << "Options:" << '\n';

    print_helper("-convert", "", "Convert inputFile (from -src) to to outputFile (from -dst)");
    print_helper("-printE57Format", "", "Print format of e57File (from -src)");
    print_helper("-buildLOD", "", "BuildLOD for OutOfCoreOctreeFile (from -src)");
    print_helper("-h", "", "Show this help");

    std::cout << "Parmameters - convert:" << '\n';

    print_helper("-input", "<sting>", "Input PointCloud file");
    print_helper("-output", "<sting>", "Output PointCloud file");
    print_helper("-res", "<float>", "Size for converting inputFile(from -src) to OutOfCoreOctreeFile(from -dst)");
    print_helper("-min", "<sting of tree floats>", "AABB min corner for converting inputFile(from -src) to OutOfCoreOctreeFile(from -dst). For example: -min \"-100 -100 -100\"");
    print_helper("-max", "<sting of tree floats>", "AABB max corner for converting inputFile(from -src) to OutOfCoreOctreeFile(from -dst). For example: -max \"100 100 100");
    print_helper("-samplePercent", "<float>", "Sample percent for build OutOfCoreOctree(from -src) LOD");
    print_helper("-voxelUnit", "<float>", "Size for converting OutOfCoreOctree(from -src) to pcdFile(from -dst)");

    print_helper("-rgb", "", "Output rgb for converting pcdFile(from -src) to plyFile(from -dst)");
    print_helper("-binary", "", "Output binary for converting pcdFile(from -src) to plyFile(from -dst)");
    print_helper("-camera", "", "Output camera for converting pcdFile(from -src) to plyFile(from -dst)");

    std::cout << "Parmameters - buildLOD:" << '\n';

    print_helper("-samplePercent", "<float>", "Sample percent for build OutOfCoreOctree(from -src) LOD");
}

void parse_arguments(int argc, char **argv)
{
    if (pcl::console::find_switch(argc, argv, "-h"))
    {
        print_help();
        return;
    }

    if (pcl::console::find_switch(argc, argv, "-convert"))
    {
        std::string _input_file_path{""};
        std::string _output_file_path{""};
        pcl::console::parse_argument(argc, argv, "-input", _input_file_path);
        pcl::console::parse_argument(argc, argv, "-output", _output_file_path);

        fs::path input_file_path{_input_file_path};
        fs::path output_file_path{_output_file_path};
        
        FileType input_file_type = get_file_type(input_file_path);
        FileType output_file_type = get_file_type(output_file_path);
        
        if(!are_input_output_valid_match(input_file_type, output_file_type))
        {
            throw std::runtime_error{"Unsupported conversion from " + file_type_to_string(input_file_type) + " to " + file_type_to_string(output_file_type)};
        }

        std::cout << "Parmameters - input: " << input_file_path << std::endl;
        std::cout << "Parmameters - ouput: " << output_file_path << std::endl;
    }
}

#endif //__E57_UTILS_H__