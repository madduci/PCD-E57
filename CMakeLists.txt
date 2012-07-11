cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

PROJECT(E57Converter)

SET(CMAKE_BUILD_TYPE Release)

# Private Xerces find module
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
# find and setup PCL 1.4 for this project
FIND_PACKAGE(PCL 1.5 REQUIRED)

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

add_definitions(-DBOOST_ALL_NO_LIB -DXERCES_STATIC_LIBRARY)
#~ #--------------------------------------------------------------------------------
#~ # E57 3rd Party libraries
#~ #--------------------------------------------------------------------------------
#~ 

add_library( E57LIB STATIC
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
SET( CP_SRC
  main.cpp 
)


#--------------------------------------------------------------------------------
# another list, this time it includes all header files that should be treated with moc
#--------------------------------------------------------------------------------
SET( CP_HDRS
  e57.h 
)

SET_SOURCE_FILES_PROPERTIES( ${CP_SRC} ${CP_MOC_HDRS}  PROPERTIES LANGUAGE CXX )

INCLUDE_DIRECTORIES (
  # Look in the cmake build directory (some generated headers could be there)
  ${CMAKE_CURRENT_BINARY_DIR} 
)
#for PointCloudLibrary too
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS}
	${XML_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

#--------------------------------------------------------------------------------
#  CMake's way of creating an executable
ADD_EXECUTABLE( e57_2_pcd MACOSX_BUNDLE WIN32
  ${CP_SRC}
  ${CP__HDRS} 
  
)


TARGET_LINK_LIBRARIES ( e57_2_pcd   ${PCL_LIBRARIES} 
									  E57LIB
									  ${XML_LIBRARIES}
									  ${CMAKE_THREAD_LIBS_INIT}
									  )

# directories to look for dependencies
SET(DIRS PCL_LIBRARY_DIRS}  ${XML_INCLUDE_DIRS})
