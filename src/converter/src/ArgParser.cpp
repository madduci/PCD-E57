#include "ArgParser.h"
#include "FileType.h"

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <iostream>
#include <iomanip>

using namespace converter::app;

constexpr int CMD_SPACE{5};
constexpr int PARMS_SPACE{30};

constexpr auto print_helper = [](std::string cmd, std::string param, std::string description) {
    std::cout << std::right << std::setw(CMD_SPACE) << cmd
              << std::left << std::setw(PARMS_SPACE) << param << '\n'
              << std::left << std::setw(CMD_SPACE) << ""
              << "\t" << description << '\n'
              << '\n';
};

ArgParser::ArgParser(int argc, char **argv)
{
    if (argc <= 1)
    {
        print_help();
        throw std::runtime_error{"No argument passed"};
    }

    parse_arguments(argc, argv);
}

const std::unordered_map<std::string, std::string> &ArgParser::getParameters() const noexcept
{
    return parameters;
}

void ArgParser::print_help()
{
    std::cout << "Options:" << '\n';

    print_helper("-convert", "", "Convert inputFile (from -src) to to outputFile (from -dst)");
    print_helper("-printE57Format", "", "Print format of e57File (from -src)");
    print_helper("-buildLOD", "", "BuildLOD for OutOfCoreOctreeFile (from -src)");
    print_helper("-h", "", "Show this help");

    std::cout << "Parmameters - convert:" << '\n';

    print_helper("-input", "<sting>", "Input PointCloud file");
    print_helper("-output", "<sting>", "Output PointCloud file");
    print_helper("-samplePercent", "<float>", "Sample percent for build OutOfCoreOctree(from -src) LOD");
    print_helper("-voxelUnit", "<float>", "Size for converting OutOfCoreOctree(from -src) to pcdFile(from -dst)");
}

void ArgParser::parse_arguments(int argc, char **argv)
{
    if (pcl::console::find_switch(argc, argv, "-h"))
    {
        print_help();
        return;
    }

    if (pcl::console::find_switch(argc, argv, "-convert"))
    {
        parse_conversion_arguments(argc, argv);
    }
}

void ArgParser::parse_conversion_arguments(int argc, char **argv)
{
    std::string input_file_path_string{""};
    std::string output_file_path_string{""};
    pcl::console::parse_argument(argc, argv, "-input", input_file_path_string);
    pcl::console::parse_argument(argc, argv, "-output", output_file_path_string);

    fs::path input_file_path{input_file_path_string};
    fs::path output_file_path{output_file_path_string};

    FileType input_file_type = get_file_type(input_file_path);
    FileType output_file_type = get_file_type(output_file_path);

    if (!are_input_output_valid_match(input_file_type, output_file_type))
    {
        throw std::runtime_error{"Unsupported conversion from " + file_type_to_string(input_file_type) + " to " + file_type_to_string(output_file_type)};
    }

    std::cout << "Parameters - input: " << input_file_path << '\n';
    std::cout << "Parameters - ouput: " << output_file_path << '\n';

    parameters.emplace(std::make_pair("operation", "convert"));
    parameters.emplace(std::make_pair("input", input_file_path_string));
    parameters.emplace(std::make_pair("input_type", file_type_to_string(input_file_type)));
    parameters.emplace(std::make_pair("output", output_file_path_string));
    parameters.emplace(std::make_pair("output_type", file_type_to_string(output_file_type)));
}
