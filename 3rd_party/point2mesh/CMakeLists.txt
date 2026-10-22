#cmake_minimum_required(VERSION 2.8.11)
project(point2mesh)


function(aux_source_directories var folder)
    set(sources)
    foreach(dir ${ARGN})
        file(GLOB _sources RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "${dir}/*.[ch]" "${dir}/*.[ch]pp")
        list(APPEND sources ${_sources})
    endforeach()
    
    if( NOT folder STREQUAL "")
        source_group(${folder} FILES ${sources})
    endif()

    #message("\nDEBUG: aux_source_directories: current_source_dir=${CMAKE_CURRENT_SOURCE_DIR} dirs=${ARGN}):\n${sources}\n")
    set(${var} ${${var}} ${sources} PARENT_SCOPE)
endfunction()

include_directories(${PROJECT_SOURCE_DIR}/..)

add_subdirectory(third_party)

aux_source_directories(SOURCES "Source Files"           .)
aux_source_directories(SOURCES "Source Files\\basic"    basic)
aux_source_directories(SOURCES "Source Files\\numerics" numerics)
aux_source_directories(SOURCES "Source Files\\mesh"     mesh)
aux_source_directories(SOURCES "Source Files\\delaunay" delaunay)
aux_source_directories(SOURCES "Source Files\\voronoi"  voronoi)
aux_source_directories(SOURCES "Source Files\\points"   points)
aux_source_directories(SOURCES "Source Files\\api"      api)
aux_source_directories(SOURCES "Source Files\\NL"       NL)
aux_source_directories(SOURCES "Source Files\\image"    image)
aux_source_directories(SOURCES "Source Files\\parameterization" parameterization)
aux_source_directories(SOURCES "Source Files\\bibliography" bibliography)


# Parallel Delaunay now works on both Windows and Unix !
add_definitions(-DGEOGRAM_WITH_PDEL)

# Copy the default version info to the build directory
configure_file(basic/version.h.in version.h @ONLY)
list(APPEND SOURCES version.h)
set_source_files_properties(version.h PROPERTIES GENERATED true)

include_directories(${PROJECT_BINARY_DIR}/..)

add_library(point2mesh ${SOURCES} $<TARGET_OBJECTS:point2mesh_third_party>)

set_target_properties(point2mesh PROPERTIES FOLDER "3rd_party")

target_link_libraries(point2mesh psapi)

