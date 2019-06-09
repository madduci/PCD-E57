
#-----------------------------
# Project Settigs 
#-----------------------------
project(converter VERSION 0.2.0 LANGUAGES CXX)


message( STATUS "-------------------------------------------------------------------------------" )
message( STATUS "Building converter")
message( STATUS "-------------------------------------------------------------------------------" )

# Define library target
add_executable(e57conv
  src/main.cpp
  src/ArgParser.cpp
)
# Specify library properties
target_include_directories(e57conv PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/libE57/include>
		$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/libconverter/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

set_target_properties(e57conv PROPERTIES CXX_STANDARD 17)

target_compile_options(e57conv PUBLIC -pedantic -Wall -Wextra -Wfloat-equal -O2)

target_link_libraries(e57conv converter e57 pcl boost)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS e57conv 
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
