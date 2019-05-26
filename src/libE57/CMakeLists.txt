
#-----------------------------
# Project Settings
#-----------------------------
project(libE57 VERSION 1.1.312 LANGUAGES CXX)


message( STATUS "-------------------------------------------------------------------------------" )
message( STATUS "Building libE57")
message( STATUS "-------------------------------------------------------------------------------" )

# Define library target
add_library( e57 STATIC
  src/E57Simple.cpp
	src/E57SimpleImpl.cpp
	src/E57Foundation.cpp
	src/E57FoundationImpl.cpp
	src/time_conversion.c
)

message( STATUS "-------------------------------------------------------------------------------" )
message( STATUS "Looking for Xerces and ICU")
message( STATUS "-------------------------------------------------------------------------------" )

# Include Xerces as imported target
set(Xerces_USE_STATIC_LIBS On)
find_package(Xerces REQUIRED)
add_library(xerces SHARED IMPORTED)
set(XML_LIBRARIES ${Xerces_LIBRARY})
set(XML_INCLUDE_DIRS ${Xerces_INCLUDE_DIR})
if (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	target_compile_definitions(e57 PUBLIC LINUX)
	find_package(ICU REQUIRED)
	set(XML_LIBRARIES ${XML_LIBRARIES} ${ICU_LIBRARIES})
	set(XML_INCLUDE_DIRS ${XML_INCLUDE_DIRS} ${ICU_INCLUDE_DIRS})
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
	target_compile_definitions(e57 PUBLIC WINDOWS)
endif()

set_target_properties(xerces 
PROPERTIES 
	"INTEFACE_INCLUDE_DIRECTORIES" "${XML_INCLUDE_DIRS}"
	"IMPORTED_LOCATION" "${XML_LIBRARIES}")
set(CDEFS "BOOST_ALL_NO_LIB\;XERCES_STATIC_LIBRARY\;E57_REFIMPL_REVISION_ID=${${PROJECT_NAME}_TAG}\;")

# Specify library properties
target_include_directories(e57 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_compile_definitions(e57 PRIVATE ${CDEFS})

target_compile_options(e57 PRIVATE -O2)

target_link_libraries(e57 xerces boost)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS e57 EXPORT E57Config
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT E57Config DESTINATION share/libE57/cmake)

# This makes the project importable from the build directory
export(TARGETS e57 FILE E57Config.cmake)
