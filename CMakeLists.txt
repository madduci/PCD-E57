cmake_minimum_required(VERSION 3.9)

#-----------------------------
# Project Settings
#-----------------------------
SET(PROJECTNAME "E57Converter")
PROJECT(${PROJECTNAME} C CXX)
SET( ${PROJECT_NAME}_MAJOR_VERSION 0 )
SET( ${PROJECT_NAME}_MINOR_VERSION 2 )
SET( ${PROJECT_NAME}_PATCH_LEVEL 0 )
SET( ${PROJECT_NAME}_VERSION_STRING
        ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_LEVEL})
SET(CMAKE_BUILD_TYPE Release)

# Enables C++17 features
IF(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	SET(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wpedantic")
ENDIF()


#-----------------------------
# Define default destinations
#-----------------------------
SET(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

#-----------------------------
# Enable Relative path
#-----------------------------
# don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH FALSE)
# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

message( STATUS "-------------------------------------------------------------------------------" )
message( STATUS "INSTALL PATH")
message( STATUS "-------------------------------------------------------------------------------" )
message( STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message( STATUS "CMAKE_INSTALL_RPATH: ${CMAKE_INSTALL_RPATH}")


message( STATUS "-------------------------------------------------------------------------------" )
message( STATUS "Looking for Dependencies")
message( STATUS "-------------------------------------------------------------------------------" )

# Private Xerces find module
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
IF(UNIX)
INCLUDE(${CMAKE_MODULE_PATH}FindXerces.cmake)
INCLUDE(${CMAKE_MODULE_PATH}FindICU.cmake)
ENDIF(UNIX)

SET(Xerces_USE_STATIC_LIBS On)
FIND_PACKAGE(Xerces REQUIRED)
if(Xerces_FOUND)
	set(XML_LIBRARIES ${Xerces_LIBRARY})
	set(XML_INCLUDE_DIRS ${Xerces_INCLUDE_DIR})
	message(STATUS "Xerces Library (include: ${Xerces_LIBRARY})")
	if (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	    add_definitions(-DLINUX)
	    find_package(ICU REQUIRED)
	    set(XML_LIBRARIES ${XML_LIBRARIES} ${ICU_LIBRARIES})
	    set(XML_INCLUDE_DIRS ${XML_INCLUDE_DIRS} ${ICU_INCLUDE_DIRS})
	elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
		add_definitions(-DWINDOWS)
	endif()
else (NOT Xerces_FOUND)
	message(FATAL_ERROR "Xerces Library not found")
endif (Xerces_FOUND)

set(CDEFS "XERCES_STATIC_LIBRARY\;E57_REFIMPL_REVISION_ID=${${PROJECT_NAME}_TAG}\;")

# find and setup PCL 1.6.0 for this project
FIND_PACKAGE(PCL 1.6.0 REQUIRED)

add_definitions(-DBOOST_ALL_NO_LIB -DXERCES_STATIC_LIBRARY)

#--------------------------------------------------------------------------------
# Building libE57
#--------------------------------------------------------------------------------

add_library( E57LIB
	E57/E57Simple.h
	E57/E57SimpleImpl.h
  E57/E57Simple.cpp
	E57/E57SimpleImpl.cpp
	E57/E57Foundation.h
	E57/E57FoundationImpl.h
	E57/E57Foundation.cpp
	E57/E57FoundationImpl.cpp
	E57/time_conversion.h
	E57/time_conversion.c
	E57/gnss_error.h
	E57/basictypes.h
	E57/constants.h
)

set_target_properties( E57LIB
    PROPERTIES COMPILE_DEFINITIONS ${CDEFS}
)

install(
    FILES
        E57/E57Simple.h
        E57/E57Foundation.h
    DESTINATION
        include/e57
)

install(
    TARGETS
        E57LIB
    ARCHIVE DESTINATION
        lib
)

set(E57_SOURCES
	E57/E57Simple.cpp
	E57/E57SimpleImpl.cpp
	E57/E57Foundation.cpp
	E57/E57FoundationImpl.cpp
	E57/time_conversion.c
)
	
set(E57_INCLUDES 
	E57/E57Simple.h
	E57/E57SimpleImpl.h
	E57/E57Foundation.h
	E57/E57FoundationImpl.h
	E57/time_conversion.h
	E57/gnss_error.h
	E57/basictypes.h
	E57/constants.h
)
	
#--------------------------------------------------------------------------------
# This is the list of source files that need to be compiled
#--------------------------------------------------------------------------------
SET( PROJECT_SRC
	main.cpp 
	e57.cpp 
)

SET( PROJECT_HDRS
	e57.h 
  e57.h 
	e57.h 
  e57.h 
	e57.h 
	utils.h 
)

INCLUDE_DIRECTORIES (
  # Look in the cmake build directory (some generated headers could be there)
  ${CMAKE_CURRENT_BINARY_DIR} 
)

#for PointCloudLibrary too
INCLUDE_DIRECTORIES(
    ${PCL_INCLUDE_DIRS}
    ${XML_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

#--------------------------------------------------------------------------------
#  CMake's way of creating an executable
ADD_EXECUTABLE(e57_converter
  ${PROJECT_SRC}
  ${PROJECT_HDRS}
)

TARGET_LINK_LIBRARIES ( e57_converter
    ${PCL_LIBRARIES}
    E57LIB
    ${XML_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
