#ifndef __ARG_PARSER_H__
#define __ARG_PARSER_H__

#include <unordered_map>
#include <string>
namespace converter::app
{
class ArgParser
{
public:
    ArgParser(int argc, char **argv);

    [[nodiscard]] const std::unordered_map<std::string, std::string> &getParameters() const noexcept;

private:
    std::unordered_map<std::string, std::string> parameters{};
    void print_help();
    void parse_arguments(int argc, char **argv);
    void parse_conversion_arguments(int argc, char **argv);
};
} // namespace converter::app

#endif //__ARG_PARSER_H__
