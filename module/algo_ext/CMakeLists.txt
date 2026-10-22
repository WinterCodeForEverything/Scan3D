cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


set(${PROJECT_NAME}_HEADERS
        duplicated_faces.h
        mesh_surfacer.h
        self_intersection.h
        )

set(${PROJECT_NAME}_SOURCES
        duplicated_faces.cpp
        mesh_surfacer.cpp
        self_intersection.cpp
        )


add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "module")

target_include_directories(${PROJECT_NAME} PRIVATE ${3DMODEL_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${3DMODEL_THIRD_PARTY})
target_include_directories(${PROJECT_NAME} PRIVATE ${3DMODEL_glog_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} core)

include(../../cmake/UseCGAL.cmake)


