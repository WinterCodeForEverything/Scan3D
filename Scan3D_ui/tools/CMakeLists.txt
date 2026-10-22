cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


set(${PROJECT_NAME}_HEADERS
        canvas.h
        tool.h
        tool_manager.h
        tool_segmentation_surface_mesh.h
        tool_selection_point_cloud.h
        tool_selection_surface_mesh.h
        )

set(${PROJECT_NAME}_SOURCES
        tool.cpp
        tool_manager.cpp
        tool_segmentation_surface_mesh.cpp
        tool_selection_point_cloud.cpp
        tool_selection_surface_mesh.cpp
        )

add_library(${PROJECT_NAME} STATIC
        ${${PROJECT_NAME}_SOURCES}
        ${${PROJECT_NAME}_HEADERS}
        )

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "module")

target_include_directories(${PROJECT_NAME} PRIVATE ${3DMODEL_INCLUDE_DIR})

set(UI_DIR ${3DMODEL_ROOT}/Scan3D_ui)
target_include_directories(${PROJECT_NAME} PRIVATE ${UI_DIR})

target_link_libraries(${PROJECT_NAME} core util renderer gui)

target_compile_definitions(${PROJECT_NAME} PRIVATE GLEW_STATIC)

if (MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_DEPRECATE)
endif ()
