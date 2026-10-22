#ifndef SCAN3D_FILEIO_RESOURCES_H
#define SCAN3D_FILEIO_RESOURCES_H


#include <module/core/types.h>


namespace scan3d {

    namespace resource {

        // the data representing a bunny model.
        // the vertices
        extern const std::vector<vec3>    bunny_vertices;
        // the triangle faces (each consecutive 3 values denote the vertex indices of a triangle)
        extern const std::vector<unsigned int>     bunny_indices;

        // a 1D array store a gradually changing colors which each consecutive 3 elements represent a RGB color.
        extern const std::vector<unsigned char> cold_warm_colors;

		// resource directory (containing color maps, shaders, textures, etc.)
        extern std::string directory();

    } // namespace resource

} // namespace scan3d


#endif // SCAN3D_FILEIO_RESOURCES_H
