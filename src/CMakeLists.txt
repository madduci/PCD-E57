cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

#-----------------------------
# Project Settings
#-----------------------------
project(E57Converter VERSION 0.2 LANGUAGES CXX)
# install libraries into correct locations on all platforms
include(GNUInstallDirs)

#-----------------------------
# Define default destinations
#-----------------------------
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

#-----------------------------
# Enable Relative path
#-----------------------------
# don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)
# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")

message( STATUS "-------------------------------------------------------------------------------" )
message( STATUS "INSTALL PATH")
message( STATUS "-------------------------------------------------------------------------------" )
message( STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message( STATUS "CMAKE_INSTALL_RPATH: ${CMAKE_INSTALL_RPATH}")


message( STATUS "-------------------------------------------------------------------------------" )
message( STATUS "Setup CMake Modules")
message( STATUS "-------------------------------------------------------------------------------" )

# Private Xerces find module
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
if(UNIX)
	include(${CMAKE_MODULE_PATH}FindXerces.cmake)
	include(${CMAKE_MODULE_PATH}FindICU.cmake)
endif(UNIX)

# Include Boost as an imported target
find_package(Boost 1.40.0 REQUIRED)
add_library(boost INTERFACE IMPORTED)
set_property(TARGET boost PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})

find_package(PCL 1.6.0 REQUIRED)
add_library(pcl INTERFACE IMPORTED)
set_property(TARGET pcl PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${PCL_INCLUDE_DIRS})

add_subdirectory(libE57)
add_subdirectory(libconverter)
add_subdirectory(converter)

#--------------------------------------------------------------------------------
# Building libE57
#--------------------------------------------------------------------------------

# add_library( e57 STATIC
# 	E57/E57Simple.h
# 	E57/E57SimpleImpl.h
#   E57/E57Simple.cpp
# 	E57/E57SimpleImpl.cpp
# 	E57/E57Foundation.h
# 	E57/E57FoundationImpl.h
# 	E57/E57Foundation.cpp
# 	E57/E57FoundationImpl.cpp
# 	E57/time_conversion.h
# 	E57/time_conversion.c
# 	E57/gnss_error.h
# 	E57/basictypes.h
# 	E57/constants.h
# )

# set_target_properties( e57
#     PROPERTIES COMPILE_DEFINITIONS ${CDEFS}
# )

# set_property(TARGET e57 PROPERTY CXX_STANDARD 17)

# target_include_directories(e57 PRIVATE ${XML_INCLUDE_DIRS})

# target_link_libraries ( e57
#     ${XML_LIBRARIES}
# )

# #--------------------------------------------------------------------------------
# # This is the list of source files that need to be compiled
# #--------------------------------------------------------------------------------
# set( PROJECT_SRC
# 	main.cpp 
# 	e57.cpp 
# )

# set( PROJECT_HDRS
# 	e57.h 
# 	utils.h 
# )

# include_directories (
#   # Look in the cmake build directory (some generated headers could be there)
#   ${CMAKE_CURRENT_BINARY_DIR} 
# )

# #for PointCloudLibrary too
# include_directories(
#     ${PCL_INCLUDE_DIRS}
# 		${Boost_INCLUDE_DIR}
# )

# #--------------------------------------------------------------------------------
# #  CMake's way of creating an executable
# add_executable(e57_converter
#   ${PROJECT_SRC}
# 	${PROJECT_HDRS}
# )

# set_property(TARGET e57_converter PROPERTY CXX_STANDARD 17)

# target_compile_options(e57_converter PRIVATE -pedantic -Wall -Wextra -Wfloat-equal -O2)

# target_link_libraries ( e57_converter
#     ${PCL_LIBRARIES}
#     ${XML_LIBRARIES}
# 		${CMAKE_THREAD_LIBS_INIT}
# 		e57
# )
