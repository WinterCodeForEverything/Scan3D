cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


set(${PROJECT_NAME}_HEADERS
        extrusion.h
		nrutil.h
        surface_mesh_geometry.h
		svdSolveEquation.h
        gaussian_noise.h
        point_cloud_normals.h
        point_cloud_poisson_reconstruction.h
		point_cloud_delaunay_reconstruction.h
        point_cloud_simplification.h
		point_cloud_filter_utils.h
		point_cloud_filter_radius.h
		point_cloud_curvation_estimate.h
		point_cloud_feature_FPFH.h
		point_cloud_bilateral_filter.h
		point_cloud_registration.h
		surface_mesh_components.h
		surface_mesh_enumerator.h
		surface_mesh_curvature.h
		surface_mesh_features.h
		surface_mesh_remeshing.h
		surface_mesh_fixing.h
		surface_mesh_texturing.h
		surface_mesh_texturing_interface.h
        tessellator.h
        text_mesher.h
        triangle_mesh_kdtree.h
		
        registration/registration_c_utils.hpp
        registration/registration_hash_murmur.hpp
        registration/registration_hash_murmur64.hpp
        registration/registration_ObjectOpse.h
        registration/registration_ppf_helpers.hpp
        registration/registration_PPF_Match.h
        registration/registration_stdafx.h
        registration/registration_t_hash_int.hpp	
        registration/registration_IcpOptimizer.h
        ICP_Princeton/KDtree.h
        ICP_Princeton/ICP.h
        ICP_Princeton/TriMesh.h
        ICP_Princeton/TriMesh_algo.h
        )
		


set(${PROJECT_NAME}_SOURCES
        extrusion.cpp
        surface_mesh_geometry.cpp
        gaussian_noise.cpp
        point_cloud_normals.cpp
        point_cloud_poisson_reconstruction.cpp
		point_cloud_delaunay_reconstruction.cpp
        point_cloud_simplification.cpp
		point_cloud_filter_utils.cpp
		point_cloud_filter_radius.cpp
		point_cloud_curvation_estimate.cpp
		point_cloud_feature_FPFH.cpp
		point_cloud_bilateral_filter.cpp
		point_cloud_registration.cpp
		surface_mesh_components.cpp
		surface_mesh_enumerator.cpp
		surface_mesh_curvature.cpp
		surface_mesh_features.cpp
		surface_mesh_remeshing.cpp
		surface_mesh_fixing.cpp
		surface_mesh_texturing.cpp
		surface_mesh_texturing_interface.cpp
        tessellator.cpp
        text_mesher.cpp
        triangle_mesh_kdtree.cpp

        registration/registration_ObjectOpse.cpp
        registration/registration_ppf_helpers.cpp
        registration/registration_PPF_Match.cpp
        registration/registration_t_hash_int.cpp
        registration/registration_IcpOptimizer.cpp
		
        ICP_Princeton/KDtree.cpp
        ICP_Princeton/ICP.cpp
		ICP_Princeton/TriMesh_bounding.cpp
        ICP_Princeton/TriMesh_connectivity.cpp
        ICP_Princeton/TriMesh_curvature.cpp
        ICP_Princeton/TriMesh_grid.cpp
        ICP_Princeton/TriMesh_io.cpp
        ICP_Princeton/TriMesh_normal.cpp
        ICP_Princeton/TriMesh_pointareas.cpp
        ICP_Princeton/TriMesh_stats.cpp
        ICP_Princeton/TriMesh_tstrips.cpp
        ICP_Princeton/remove.cpp
        ICP_Princeton/reorder_verts.cpp
        )


add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "module")

if (MSVC)
	add_definitions(/bigobj)
	add_definitions(-D_OPENMP)
endif(MSVC)

target_include_directories(${PROJECT_NAME} PRIVATE ${3DMODEL_INCLUDE_DIR})
#target_include_directories(${PROJECT_NAME} PRIVATE ${3DMODEL_THIRD_PARTY}/RANSAC-1.1)
target_include_directories(${PROJECT_NAME} PRIVATE ${3DMODEL_THIRD_PARTY})
target_include_directories(${PROJECT_NAME} PRIVATE ${3DMODEL_THIRD_PARTY}/MVS)

target_link_libraries(${PROJECT_NAME} core util kdtree  triangle libtess poisson_recon-9.0.1 point2mesh MVS MeshFix )   # RANSAC-1.1 tetgen

set(EIGEN_SOURCE_DIR ${3DMODEL_THIRD_PARTY}/eigen)
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN_SOURCE_DIR})

# It's "Boost", not "BOOST" or "boost". Case matters.
find_package(Boost ${SYSTEM_PACKAGE_REQUIRED} COMPONENTS iostreams system serialization)
if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
	link_directories(${Boost_LIBRARY_DIRS})
	ADD_DEFINITIONS(${Boost_DEFINITIONS} -D_USE_BOOST)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_BOOST)
	SET(_USE_BOOST TRUE)
endif ()

FIND_PACKAGE(Eigen3 ${SYSTEM_PACKAGE_REQUIRED})
if(EIGEN3_FOUND)
	INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})
	ADD_DEFINITIONS(${EIGEN3_DEFINITIONS} -D_USE_EIGEN)
	SET(_USE_EIGEN TRUE)
endif()

FIND_PACKAGE(OpenCV ${SYSTEM_PACKAGE_REQUIRED})
if(OpenCV_FOUND)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
	ADD_DEFINITIONS(${OpenCV_DEFINITIONS})
	SET(_USE_OPENCV TRUE)
	MESSAGE(STATUS "OpenCV ${OpenCV_VERSION} found (include: ${OpenCV_INCLUDE_DIRS})")
else()
	MESSAGE("-- Can't find OpenCV. Please specify OpenCV directory using OpenCV_DIR variable")
endif()