#ifndef __UTILS_H__
#define __UTILS_H__

/*
************************ Last revision of this file ***********************
* $Author:: Michele Adduci
* $LastChangedDate:: 25/05/2019
**************************************************************************
*/

#include <cassert>
#include <iostream>
#include <string>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>

/*#ifdef __cpp_lib_filesystem
#include <filesystem>
using boost::filesystem = std::filesystem;
#elif __cpp_lib_experimental_filesystem
#include <experimental/filesystem>
using boost::filesystem = std::experimental::filesystem;
#else*/
#include <boost/filesystem.hpp>

//#endif

constexpr int CMD_SPACE{20};
constexpr int PARMS_SPACE{40};

using boost::filesystem = std::experimental::filesystem;

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
}

std::string
file_type_to_string(FileType t)
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
        return "UNKNOWN";
    }
}

bool is_valid_extension(boost::filesystem::path &file_path, FileType ext)
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

FileType get_file_type(boost::filesystem::path &file_path)
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
        return FileType::UNKNOWN;
}

void print_help()
{
    std::cout << "Options:" << '\n';

    print_helper("-convert", "", "Convert inputFile (from -src) to to outputFile (from -dst)");
    print_helper("-printE57Format", "", "Print format of e57File (from -src)");
    print_helper("-buildLOD", "", "BuildLOD for OutOfCoreOctreeFile (from -src)");
    print_helper("-h", "", "Show this help");

    std::cout << "Parmameters - convert:" << '\n';

    print_helper("-src", "<sting>", "Input PointCloud file");
    print_helper("-dst", "<sting>", "Output PointCloud file");
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
    if (pcl::console::find_switch(argc, arg, "-h"))
    {
        print_help();
        return;
    }

    if (pcl::console::find_switch(argc, argv, "-convert"))
    {
        std::string _src_file_path{""};
        std::string _dst_file_path{""};
        pcl::console::parse_argument(argc, argv, "-src", _src_file_path);
        pcl::console::parse_argument(argc, argv, "-dst", _dst_file_path);

        boost::filesystem::path src_file_path{_src_file_path};
        boost::filesystem::path dst_file_path{_dst_file_path};
        FileType srcFileType = get_file_type(src_file_path);
        FileType dstFileType = get_file_type(dst_file_path);

        std::cout << "Parmameters - src: " << src_file_path << std::endl;
        std::cout << "Parmameters - dst: " << dst_file_path << std::endl;
    }
}

#endif //__UTILS_H__