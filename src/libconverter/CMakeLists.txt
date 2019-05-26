
#-----------------------------
# Project Settigs 
#-----------------------------
project(libconverter VERSION 0.2.0 LANGUAGES CXX)

message( STATUS "-------------------------------------------------------------------------------" )
message( STATUS "Building libconverter")
message( STATUS "-------------------------------------------------------------------------------" )

# Define library target
add_library(converter
  src/e57.cpp
)
# Specify library properties
target_include_directories(converter PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/libE57/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

set_target_properties(converter PROPERTIES CXX_STANDARD 17)

target_compile_options(converter PRIVATE -pedantic -Wall -Wextra -Wfloat-equal -O2)

target_link_libraries(converter e57 pcl boost)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS converter EXPORT LibConverterConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
install(EXPORT LibConverterConfig DESTINATION share/libconverter/cmake)

# This makes the project importable from the build directory
export(TARGETS converter FILE LibConverterConfig.cmake)
