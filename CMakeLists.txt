cmake_minimum_required(VERSION 3.1)


SET(CMAKE_TOOLCHAIN_FILE "Toolchain file" CACHE FILEPATH "select vcpkg-master/scripts/buildsystems/vcpkg.cmake" )
#SET(CMAKE_TOOLCHAIN_FILE "E:\\3rdParty\\file\\vcpkg-master\\scripts\\buildsystems\\vcpkg.cmake")

################################################################################

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

set (PROJECT_MAJOR 2)
set (PROJECT_MINOR 2)
set (PROJECT_PATCH 0)
set (PROJECT_VERSION "${PROJECT_MAJOR}.${PROJECT_MINOR}.${PROJECT_PATCH}")


################################################################################

# One shouldn't generate the BUILD project directly in the SOURCES folder!
if ( ${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR} )
        if ( NOT SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED )
                message(FATAL_ERROR
                    "It is not advised to BUILD the binaries directly in the SOURCE folder!\n If
                    you want to proceed with this option, just CONFIGURE the project once again" )
                set( SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED TRUE )
        endif()
endif()

################################################################################

#math(EXPR 3DMODEL_OS_BITS "8 * ${CMAKE_SIZEOF_VOID_P}")
#message(STATUS "3DMODEL: I'am woking on ${3DMODEL_OS_BITS}-bit ${CMAKE_SYSTEM_NAME}")
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(STATUS "3DMODEL: I'am working on a 64-bit ${CMAKE_SYSTEM_NAME}")
else()
    message(STATUS "3DMODEL: I'am working on a 32-bit ${CMAKE_SYSTEM_NAME}")
endif()

################################################################################

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set unicode
#add_definitions(-DUNICODE -D_UNICODE)

################################################################################

# Detects whether this is a top-level project
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	set(3DMODEL_TOPLEVEL_PROJECT ON)
else()
	set(3DMODEL_TOPLEVEL_PROJECT OFF)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

################################################################################

# Build Qt-based examples/applications
option(3DMODEL_ENABLE_QT  "Build UI/applications"  ON)

find_package(CGAL QUIET)
if (CGAL_FOUND)
    set(3DMODEL_HAS_CGAL TRUE)
    add_definitions(-DHAS_CGAL)
else ()
    set(3DMODEL_HAS_CGAL FALSE)
    message(WARNING "Some advanced features require CGAL but CGAL was not found. Scan3D will be built without these features. "
            "You can set 'CGAL_DIR' to the directory containing 'CGALConfig.cmake' or 'cgal-config.cmake' to help find CGAL.")
endif ()

################################################################################

find_package(OpenMP)
if (OPENMP_FOUND)
	#message("OPENMP FOUND")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()



### Configuration
set(3DMODEL_ROOT         ${CMAKE_CURRENT_LIST_DIR})
set(3DMODEL_THIRD_PARTY  ${3DMODEL_ROOT}/3rd_party)

set(3DMODEL_INCLUDE_DIR  ${3DMODEL_ROOT} ${CMAKE_CURRENT_BINARY_DIR})
set(3DMODEL_SOURCE_DIR   ${3DMODEL_ROOT})

### conditionally compile certain modules depending on libraries found on the system
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

################################################################################

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/lib)

################################################################################

add_subdirectory(3rd_party)
add_subdirectory(module)
add_subdirectory(Scan3D_ui)


################################################################################

# hide some variables that might be set in 3rd_party libraries
mark_as_advanced(FORCE BUILD_SHARED_LIBS)
mark_as_advanced(FORCE BUILD_TESTING)

################################################################################

## Resources
# Copy resources dirs into our shadow build directory
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/resources DESTINATION ${CMAKE_BINARY_DIR} )
