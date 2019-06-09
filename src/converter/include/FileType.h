#ifndef __FILE_TYPE_H__
#define __FILE_TYPE__

#include <cassert>
#include <string>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace converter::app
{
enum class FileType
{
    UNSUPPORTED,
    E57,
    PCD
};

/**
 * Returns the FileType as string representation
 */
[[nodiscard]] std::string file_type_to_string(FileType t) noexcept
{
    switch (t)
    {
    case FileType::E57:
        return "E57";
    case FileType::PCD:
        return "PCD";
    default:
        return "UNSUPPORTED";
    }
}

/**
 * Checks if the extention of a file is valid
 */ 
[[nodiscard]] bool is_valid_extension(fs::path &file_path, FileType ext) noexcept
{
    assert(ext == FileType::E57 || ext == FileType::PCD);
    if (file_path.string().back() != '/' && file_path.string().back() != '\\')
    {
        auto extension = file_path.extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(), ::toupper);
        if (extension == file_type_to_string(ext))
            return true;
    }
    return false;
}

/**
 * Returns the FileType from a path
 */
[[nodiscard]] FileType get_file_type(fs::path &file_path) noexcept
{
    if (is_valid_extension(file_path, FileType::E57))
        return FileType::E57;
    else if (is_valid_extension(file_path, FileType::PCD))
        return FileType::PCD;
    else
        return FileType::UNSUPPORTED;
}

/**
 * Check the supported type of conversion between input and output
 */
[[nodiscard]] bool are_input_output_valid_match(FileType inputExt, FileType outputExt) noexcept
{
    return (inputExt == FileType::E57 && outputExt == FileType::PCD) 
        || (inputExt == FileType::PCD && outputExt == FileType::E57);
}

} // namespace converter::app

#endif //__FILE_TYPE__
