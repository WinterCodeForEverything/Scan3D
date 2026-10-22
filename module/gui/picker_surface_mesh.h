#ifndef SCAN3D_GUI_PICKER_SURFACE_MESH_H
#define SCAN3D_GUI_PICKER_SURFACE_MESH_H

#include <module/gui/picker.h>
#include <module/core/surface_mesh.h>


namespace scan3d {

    class ShaderProgram;

    class SurfaceMeshPicker : public Picker {
    public:
        SurfaceMeshPicker(const Camera *cam);

        ~SurfaceMeshPicker();

        //------------------ sensitivity  ---------------------

        int resolution() const { return hit_resolution_; }

        void set_resolution(int r) { hit_resolution_ = r; }

        //--------------- pick a single element ---------------

        /**
         * Pick a face from a surface mesh given the cursor position.
         * @param x The cursor x-coordinate, relative to the left edge of the content area.
         * @param y The cursor y-coordinate, relative to the top edge of the content area.
         * @attention The screen point is expressed in the screen coordinate system with an origin in the upper left
         *            corner. So it doesn't necessarily correspond to a pixel on High DPI devices, e.g. a Mac with
         *            a Retina display.
         * @return The picked face.
         */
        SurfaceMesh::Face pick_face(SurfaceMesh *model, int x, int y);

        /**
         * Pick a vertex from a surface mesh given the cursor position.
         * @param x The cursor x-coordinate, relative to the left edge of the content area.
         * @param y The cursor y-coordinate, relative to the top edge of the content area.
         * @attention The screen point is expressed in the screen coordinate system with an origin in the upper left
         *            corner. So it doesn't necessarily correspond to a pixel on High DPI devices, e.g. a Mac with
         *            a Retina display.
         * @return The picked vertex.
         */
        SurfaceMesh::Vertex pick_vertex(SurfaceMesh *model, int x, int y);

        /**
         * Pick an edge from a surface mesh given the cursor position.
         * @param x The cursor x-coordinate, relative to the left edge of the content area.
         * @param y The cursor y-coordinate, relative to the top edge of the content area.
         * @attention The screen point is expressed in the screen coordinate system with an origin in the upper left
         *            corner. So it doesn't necessarily correspond to a pixel on High DPI devices, e.g. a Mac with
         *            a Retina display.
         * @return The picked halfedge.
         */
        SurfaceMesh::Halfedge pick_edge(SurfaceMesh *model, int x, int y);

        /**
         * Pick a vertex from a surface mesh given the cursor position and a known picked face.
         * @param x The cursor x-coordinate, relative to the left edge of the content area.
         * @param y The cursor y-coordinate, relative to the top edge of the content area.
         * @param picked_face The picked face.
         * @attention The screen point is expressed in the screen coordinate system with an origin in the upper left
         *            corner. So it doesn't necessarily correspond to a pixel on High DPI devices, e.g. a Mac with
         *            a Retina display.
         * @return The picked vertex.
         * @attention This method must be called after calling to pick_face(). The result is valid only if the
         *            picked_face is valid.
         */
        // @attention call this version if you already picked the face
        SurfaceMesh::Vertex pick_vertex(SurfaceMesh *model, SurfaceMesh::Face picked_face, int x, int y);

        /**
         * Pick an edge from a surface mesh given the cursor position and a known picked face.
         * @param x The cursor x-coordinate, relative to the left edge of the content area.
         * @param y The cursor y-coordinate, relative to the top edge of the content area.
         * @param picked_face The picked face.
         * @attention The screen point is expressed in the screen coordinate system with an origin in the upper left
         *            corner. So it doesn't necessarily correspond to a pixel on High DPI devices, e.g. a Mac with
         *            a Retina display.
         * @return The picked halfedge.
         * @attention This method must be called after calling to pick_face(). The result is valid only if the
         *            picked_face is valid.
         */
        SurfaceMesh::Halfedge pick_edge(SurfaceMesh *model, SurfaceMesh::Face picked_face, int x, int y);

        //-------------------- query after picking ----------------------

        /**
         * Query the previously picked face.
         * @return The previously picked face.
         * @attention This method must be called after calling to one of the above pick element methods. The results is
         *            valid only if a face has been picked.
         */
        SurfaceMesh::Face picked_face() const;

        /**
         * Query the coordinate of the previously picked position, which is the intersection between the picking line
         * and the picked face.
         * @param x The cursor x-coordinate, relative to the left edge of the content area.
         * @param y The cursor y-coordinate, relative to the top edge of the content area.
         * @param picked_face The picked face.
         * @return The xyz coordinate of the picked position.
         * @attention This method must be called after calling to pick_face(). The result is valid only if the
         *            picked_face is valid.
         */
        vec3 picked_point(SurfaceMesh *model, SurfaceMesh::Face picked_face, int x, int y) const;


        //------------------ multiple selection of faces ------------------

        /**
         * @brief Pick faces of a surface mesh by a rectangle. The selected faces will be marked in face property
         * "f:select".
         * @param rect The rectangle region.
         * @param deselect True to perform an inverse operation.
         * @return The number of faces selected during this operation (despite their previous status).
         */
        int pick_faces(SurfaceMesh *model, const Rect &rect, bool deselect);

        /**
         * @brief Pick faces of a surface mesh by a polygon/lasso. The selected faces will be marked in face property
         * "f:select".
         * @param plg The polygon region.
         * @param deselect True to perform an inverse operation.
         * @return The number of faces selected during this operation (despite their previous status).
         */
        int pick_faces(SurfaceMesh *model, const Polygon2 &plg, bool deselect);

    private:
        // selection implemented in GPU (using shader program)
        SurfaceMesh::Face pick_face_gpu(SurfaceMesh *model, int x, int y);

        // selection implemented in CPU (with OpenMP if supported)
        SurfaceMesh::Face pick_face_cpu(SurfaceMesh *model, int x, int y);

        Plane3 face_plane(SurfaceMesh *model, SurfaceMesh::Face face) const;

        /**
         * TODO: check if this works also for non-convex faces.
         */
        inline bool do_intersect(SurfaceMesh *model, SurfaceMesh::Face picked_face, const OrientedLine3 &line) const {
            // Uses Plucker coordinates (see OrientedLine)
            Sign face_sign = ZERO;
            for (auto h : model->halfedges(picked_face)) {
                auto s = model->source(h);
                auto t = model->target(h);
                const OrientedLine3 edge_line(model->position(t), model->position(s));
                Sign sign = OrientedLine3::side(line, edge_line);
                if (sign != ZERO) {
                    if (face_sign != ZERO && sign != face_sign)
                        return false;
                    face_sign = sign;
                }
            }
            return true;
        }

    private:
        ShaderProgram *program_;
        unsigned int hit_resolution_;     // in pixels
        SurfaceMesh::Face picked_face_;
    };

}


#endif  // SCAN3D_GUI_PICKER_SURFACE_MESH_H
