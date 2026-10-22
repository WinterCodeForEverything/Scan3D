#ifndef SCAN3D_ALGO_EXTRUSION_H
#define SCAN3D_ALGO_EXTRUSION_H

#include <string>

#include <module/core/types.h>


namespace scan3d {

    class SurfaceMesh;

    /**
     * Extrude a 3D surface mesh from a set of simple contours.
     * @param contours The input contours, which must be simple, i.e.,
     *      - free of intersections,
     *      - CCW contours defining the outer boundary and CW contours defining holes.
     *      Simple contours can be obtained using the tessellator.
     * @param height The height (in the Z direction) of the extruded 3D model.
     * @return The extruded surface mesh model. NULL on failure.
     */
    SurfaceMesh *extrude(const std::vector<Polygon2> &simple_contours, float height);

    /**
     * Extrude a 3D surface mesh from a set of simple contours.
     * @param contours The input contours, which must be simple, i.e.,
     *      - free of intersections,
     *      - CCW contours defining the outer boundary and CW contours defining holes.
     *      Simple contours can be obtained using the tessellator.
     * @param height The height (in the Z direction) of the extruded 3D model.
     * @return True on success and false on failure.
     */
    bool extrude(SurfaceMesh *mesh, const std::vector<Polygon2> &simple_contours, float height);

}


#endif  // SCAN3D_ALGO_EXTRUSION_H
