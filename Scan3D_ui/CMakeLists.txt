cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


add_subdirectory(tools)

include(../cmake/UseQt5.cmake)
if (NOT QT5_FOUND)
    message(WARNING "${PROJECT_NAME} requires Qt but Qt was not found. You can set 'Qt5_DIR' to the "
            "directory containing 'Qt5Config.cmake' or 'qt5-config.cmake'. "
            "Optionally, you can set the Qt5 root directory 'QT5_ROOT_PATH' to the directory "
            "containing the 'bin' folder.")
    return()
endif ()


# some algorithms may use OpenMP
include(../cmake/UseOpenMP.cmake)

set(${PROJECT_NAME}_HEADERS
        main_window.h
        paint_canvas.h

        dialogs/dialog.h
		dialogs/dialog_surface_mesh_remeshing.h
		dialogs/dialog_surface_mesh_texturing.h
        dialogs/dialog_poisson_reconstruction.h
        dialogs/dialog_properties.h
        dialogs/dialog_snapshot.h
        dialogs/dialog_point_cloud_simplification.h
		dialogs/dialog_point_cloud_statistic_filter.h
		dialogs/dialog_point_cloud_radius_filter.h  
		dialogs/dialog_point_cloud_bilateral_filter.h
		dialogs/dialog_point_cloud_curvation_estimate.h
		dialogs/dialog_point_cloud_feature_FPFH.h
		dialogs/dialog_point_cloud_delaunay_reconstruction.h
		dialogs/dialog_point_cloud_registration.h

        widgets/widget_checker_sphere.h
        widgets/widget_drawable.h
        widgets/widget_drawable_points.h
        widgets/widget_drawable_lines.h
        widgets/widget_drawable_triangles.h
        widgets/widget_global_setting.h
        widgets/widget_model_list.h
        )

set(${PROJECT_NAME}_SOURCES
        main.cpp
        main_window.cpp
        paint_canvas.cpp
        paint_canvas_snapshot.cpp

        dialogs/dialog.cpp
		dialogs/dialog_surface_mesh_remeshing.cpp
		dialogs/dialog_surface_mesh_texturing.cpp
        dialogs/dialog_poisson_reconstruction.cpp
        dialogs/dialog_properties.cpp
        dialogs/dialog_snapshot.cpp
        dialogs/dialog_point_cloud_simplification.cpp
		dialogs/dialog_point_cloud_statistic_filter.cpp
		dialogs/dialog_point_cloud_radius_filter.cpp
		dialogs/dialog_point_cloud_bilateral_filter.cpp
		dialogs/dialog_point_cloud_curvation_estimate.cpp
		dialogs/dialog_point_cloud_feature_FPFH.cpp
		dialogs/dialog_point_cloud_delaunay_reconstruction.cpp
		dialogs/dialog_point_cloud_registration.cpp

        widgets/widget_checker_sphere.cpp
        widgets/widget_drawable.cpp
        widgets/widget_drawable_points.cpp
        widgets/widget_drawable_lines.cpp
        widgets/widget_drawable_triangles.cpp
        widgets/widget_global_setting.cpp
        widgets/widget_model_list.cpp
        )

set(${PROJECT_NAME}_FORMS
        main_window.ui

		dialogs/dialog_surface_mesh_remeshing.ui
		dialogs/dialog_surface_mesh_texturing.ui
		dialogs/dialog_poisson_reconstruction.ui
        dialogs/dialog_properties.ui
        dialogs/dialog_snapshot.ui
        dialogs/dialog_point_cloud_simplification.ui
		dialogs/dialog_point_cloud_statistic_filter.ui
		dialogs/dialog_point_cloud_radius_filter.ui
		dialogs/dialog_point_cloud_bilateral_filter.ui
		dialogs/dialog_point_cloud_curvation_estimate.ui
		dialogs/dialog_point_cloud_feature_FPFH.ui
		dialogs/dialog_point_cloud_delaunay_reconstruction.ui
		dialogs/dialog_point_cloud_registration.ui

        widgets/widget_drawable_points.ui
        widgets/widget_drawable_lines.ui
        widgets/widget_drawable_triangles.ui
        widgets/widget_global_setting.ui

        )

set(${PROJECT_NAME}_RESOURCES
        3DScan.qrc
        )

if (MSVC)
    add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_HEADERS} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_FORMS} ${${PROJECT_NAME}_RESOURCES})
    # if you want to get rid of the (system) console, use
    #	add_executable( ${PROJECT_NAME} WIN32 ${${PROJECT_NAME}_HEADERS} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_FORMS} ${${PROJECT_NAME}_RESOURCES} )
    #       target_link_libraries( ${PROJECT_NAME} Qt5::WinMain )
elseif (APPLE)
    add_executable(${PROJECT_NAME} MACOSX_BUNDLE ${${PROJECT_NAME}_HEADERS} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_FORMS} ${${PROJECT_NAME}_RESOURCES})
else ()
    add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_HEADERS} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_FORMS} ${${PROJECT_NAME}_RESOURCES})
endif ()

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Scan3D_ui")

target_include_directories(${PROJECT_NAME} PRIVATE ${3DMODEL_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${3DMODEL_glog_INCLUDE_DIR})

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
target_compile_definitions(${PROJECT_NAME} PRIVATE QT_DEPRECATED_WARNINGS GLEW_NO_GLU)

if (MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_DEPRECATE)
endif ()

target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)
target_link_libraries(${PROJECT_NAME} renderer algo tools fileio)

if (3DMODEL_HAS_CGAL)
    target_link_libraries(${PROJECT_NAME} algo_ext)
	add_custom_command(
		TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} 
	)
endif ()


## Note: On macOS, high-DPI support is enabled by settings in the Info.plist file. Make sure they are present.
## For more information, see: http://doc.qt.io/qt-5/highdpi.html
## Enable high DPI support by using my Info.plist file in which I added NSHighResolutionCapable to be true.
if (APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/resources/Mac/ui.plist)
endif ()