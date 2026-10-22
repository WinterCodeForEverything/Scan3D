cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


set(${PROJECT_NAME}_HEADERS
    image_io.h
    graph_io.h
    ply_reader_writer.h
	obj_reader_judge.h
    point_cloud_io.h
    point_cloud_io_ptx.h
    point_cloud_io_vg.h
    surface_mesh_io.h
    resources.h
    )

set(${PROJECT_NAME}_SOURCES
    image_io.cpp
    graph_io.cpp
    graph_io_ply.cpp
    ply_reader_writer.cpp
	obj_reader_judge.cpp
    point_cloud_io.cpp
    point_cloud_io_bin.cpp
    point_cloud_io_las.cpp
    point_cloud_io_ply.cpp
    point_cloud_io_ptx.cpp
    point_cloud_io_vg.cpp
    point_cloud_io_xyz.cpp
	point_cloud_io_obj.cpp
	
    surface_mesh_io.cpp
    surface_mesh_io_obj.cpp
    surface_mesh_io_off.cpp
    surface_mesh_io_ply.cpp
    surface_mesh_io_poly.cpp
    surface_mesh_io_stl.cpp
	surface_mesh_io_3mf.cpp
    resources.cpp
    )

	
add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS} )

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "module")

target_include_directories(${PROJECT_NAME} PRIVATE ${3DMODEL_INCLUDE_DIR})

set(LASTOOLS_INCLUDE_DIR ${3DMODEL_THIRD_PARTY}/LAStools/LASzip/src ${3DMODEL_THIRD_PARTY}/LAStools/LASlib/inc)
target_include_directories(${PROJECT_NAME} PRIVATE ${LASTOOLS_INCLUDE_DIR})

set(LASTOOLS_INCLUDE_DIR ${3DMODEL_THIRD_PARTY}/LAStools/LASzip/src ${3DMODEL_THIRD_PARTY}/LAStools/LASlib/inc)
target_include_directories(${PROJECT_NAME} PRIVATE ${LASTOOLS_INCLUDE_DIR})

target_include_directories(${PROJECT_NAME} PRIVATE ${LIB3MF_BINDING_INCLUDE} )

target_link_libraries( ${PROJECT_NAME} core util LAStools rply lib3mf )

if (MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_DEPRECATE)
endif()
