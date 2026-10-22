################################################################################
# General Informations
################################################################################

cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

################################################################################

#if(NOT CMAKE_BUILD_TYPE)
#	message(STATUS "No build type selected, default to Release")
#	set(CMAKE_BUILD_TYPE "Release")
#endif()

################################################################################

#SET( ${PROJECT_NAME}_HEADERS
#		include/Kernel/*.h
#		include/TMesh/*.h
#) 
FILE( GLOB_RECURSE HEADERS *.h )

set(SOURCES
		src/TMesh/edge.cpp
		src/TMesh/vertex.cpp
		src/TMesh/io.cpp
		src/TMesh/triangle.cpp
		src/TMesh/tin.cpp
		src/Algorithms/detectIntersections.cpp
		src/Algorithms/holeFilling.cpp
		src/Algorithms/marchIntersections.cpp
		src/Algorithms/checkAndRepair.cpp
		src/Kernel/heap.cpp
		src/Kernel/matrix.cpp
		src/Kernel/orientation.c
		src/Kernel/list.cpp
		src/Kernel/coordinates.cpp
		src/Kernel/tmesh.cpp
		src/Kernel/graph.cpp
		src/Kernel/point.cpp
		src/Kernel/jqsort.cpp
		#src/MeshFix/meshfix.cpp
)

################################################################################

add_library(${PROJECT_NAME} STATIC ${HEADERS} ${SOURCES} )

target_include_directories(${PROJECT_NAME} PUBLIC
		include/TMesh
		include/Kernel
)

################################################################################

#if(CMAKE_SIZEOF_VOID_P EQUAL 8)
#	message(STATUS "64 bits compiler detected")
#	target_compile_definitions(MeshFix PUBLIC -DIS64BITPLATFORM)
#	set(BIN_FOLDER "bin64")
#else()
#	message(STATUS "32 bits compiler detected")
#	set(BIN_FOLDER "bin")
#endif()

################################################################################

#set_target_properties(MeshFix
#	PROPERTIES
#	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${BIN_FOLDER}"
#)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "3rd_party")
## Use C++11
#set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)
#set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD_REQUIRED ON)
