#ifndef SCAN3D_RENDERER_BUFFERS_H
#define SCAN3D_RENDERER_BUFFERS_H


#include <string>

namespace scan3d {

    class Model;
    class Graph;
    class PointCloud;
    class SurfaceMesh;
    class Drawable;
    class PointsDrawable;
    class LinesDrawable;
    class TrianglesDrawable;

    namespace buffers {

        // -------------------------------------------------------------------------------------------------------------

        /**
         * @brief Update render buffers of a drawable. Coloring determined by the drawable's coloring scheme.
         * @param model     The model.
         * @param drawable  The drawable.
         */
        void update(Model* model, Drawable* drawable);


        // PointCloud -------------------------------------------------------------------------------------------------

        /**
         * @brief Update render buffers for the default "vertices" drawable of a point cloud.
         * Coloring determined by the drawable's coloring scheme.
         * @param model     The model.
         * @param drawable  The drawable.
         */
        void update(PointCloud* model, PointsDrawable* drawable);

        /**
         * @brief Update render buffers for a vector field defined on a point cloud.
         * @param model     The model.
         * @param drawable  The drawable.
         * @param field The name of the vector field.
         * @param scale The length scale of the vectors w.r.t. (0.01 * radius) of the model's bounding sphere.
         */
        void update(PointCloud *model, LinesDrawable *drawable, const std::string& field, float scale);

        // SurfaceMesh ------------------------------------------------------------------------------------------------

        /**
         * @brief Update render buffers for the default "vertices" drawable of a surface mesh.
         * Coloring determined by the drawable's coloring scheme.
         * @param model     The model.
         * @param drawable  The drawable.
         */
         void update(SurfaceMesh* model, PointsDrawable* drawable);

        /**
         * @brief Update render buffers for the default "edges" drawable of a surface mesh.
         * Coloring determined by the drawable's coloring scheme.
         * @param model     The model.
         * @param drawable  The drawable.
         */
        void update(SurfaceMesh* model, LinesDrawable* drawable);

        /**
         * @brief Update render buffers for the default "faces" drawable of a surface mesh.
         * Coloring determined by the drawable's coloring scheme.
         * @param model     The model.
         * @param drawable  The drawable.
         */
        void update(SurfaceMesh* model, TrianglesDrawable* drawable);

        /**
         * @brief Update render buffers for a vector field defined on a surface mesh.
         * @param model     The model.
         * @param drawable  The drawable.
         * @param field The name of the vector field.
         * @param location The location where the vector is defined.
         *         - 0: on faces;
         *         - 1: on vertices.
         *         - 2: one edges
         * @param scale The scale of the vector length w.r.t. the average edge length of the surface mesh.
         */
        void update(SurfaceMesh *model, LinesDrawable *drawable, const std::string& field, int location, float scale);


        // Graph ------------------------------------------------------------------------------------------------------

        /**
         * @brief Update render buffers for the default "vertices" drawable of a graph.
         * Coloring determined by the drawable's coloring scheme.
         * @param model     The model.
         * @param drawable  The drawable.
         */
        void update(Graph* model, PointsDrawable* drawable);


        /**
         * @brief Update render buffers for the default "edges" drawable of a graph.
         * Coloring determined by the drawable's coloring scheme.
         * @param model     The model.
         * @param drawable  The drawable.
         */
        void update(Graph* model, LinesDrawable* drawable);

    }   // namespaces buffers

}   // namespaces scan3d


#endif  // SCAN3D_RENDERER_BUFFERS_H