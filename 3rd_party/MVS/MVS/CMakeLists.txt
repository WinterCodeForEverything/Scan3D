# Find required packages


#add_definitions(${VCG_DEFINITIONS})

# List sources files
FILE(GLOB PCH_C "Common.cpp")

FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

# Place Common.cpp as the first file in the list
# needed by cotire when setting PCH manually
LIST(REMOVE_ITEM LIBRARY_FILES_C ${PCH_C})
SET(LIBRARY_FILES_C "${PCH_C};${LIBRARY_FILES_C}")

cxx_library_with_type_no_pch(MVS "Libs" "" "${cxx_default}"
	${LIBRARY_FILES_C} ${LIBRARY_FILES_H}
)

# Manually set Common.h as the precompiled header
set_target_pch(MVS Common.h)

#Add_ininclude
set( VCG_ROOT "../VCG")
include_directories(${VCG_ROOT})

# Link its dependencies
TARGET_LINK_LIBRARIES(MVS PRIVATE Common Math IO )  #${CERES_LIBS} ${CGAL_LIBS} ${CUDA_CUDA_LIBRARY}

set_target_properties(MVS PROPERTIES FOLDER "MVS")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")

# Install
SET_TARGET_PROPERTIES(MVS PROPERTIES
	PUBLIC_HEADER "${LIBRARY_FILES_H}")
INSTALL(TARGETS MVS
	EXPORT TextureMeshTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
	LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
	ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
	PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/MVS" COMPONENT dev)
