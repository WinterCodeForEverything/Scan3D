#ifndef SCAN3D_RENDERER_RENDERER_H
#define SCAN3D_RENDERER_RENDERER_H


#include <string>
#include <vector>

#include <module/core/types.h>

namespace scan3d {

    class Model;
    class Graph;
    class PointCloud;
    class SurfaceMesh;
    class PointsDrawable;
    class LinesDrawable;
    class TrianglesDrawable;

    /**
     * @brief A Renderer manages the drawables (and thus the rendering) of a model.
     */
    class Renderer {
    public:
        /**
         * @brief Constructor.
         * @param model The model to which this renderer is attached.
         * @param create_drawables True to create default drawables for this model. Skip the creation if false.
         *        The supported default drawables are
         *              - PointCloud: "vertices".
         *              - SurfaceMesh: "faces", "vertices", "edges", "borders".
         *              - Graph: "vertices", "edges".
         *        These default drawables are usually sufficient for basic rendering of a model. The rendering buffers
         *        of the default drawables are also automatically updated when a model is modified. In case the default
         *        drawables don't meet a particular visualization purpose, the client code should skip the creation of
         *        the default drawables, create a necessary drawable, and specify the update function. See Drawable.
         */
        Renderer(Model *model, bool create_drawables = true);

        virtual ~Renderer();

        /**
         * @brief The model to which this renderer is attached.
         */
        Model *model() { return model_; }

        //-------------------- rendering functionalities  -----------------------

        bool is_visible() const { return visible_; }
        void set_visible(bool b) { visible_ = b; }

        bool is_selected() const { return selected_; }
        void set_selected(bool b) { selected_ = b; }

        /**
         * @brief Invalidate the rendering buffers of the model and thus update the rendering.
         * @details This method triggers an update of the rendering buffers of all the drawables of the model to which
         *      this renderer is attached. The effect is equivalent to calling the update_[xxx]_buffer() functions
         *      for all the drawables of this model.
         * @Note: To achieve better performance, it is wiser to update only the affected drawables and buffers.
         */
        void update();

        //-------------------- drawable management  -----------------------

        /**
         * Get the points drawable with a given name.
         * Return nullptr if the drawable does not exist.
         */
        PointsDrawable *get_points_drawable(const std::string &name) const;

        /**
         * Get the lines drawable with a given name.
         * Return nullptr if the drawable does not exist.
         */
        LinesDrawable *get_lines_drawable(const std::string &name) const;

        /**
         * Get the triangles drawable with a given name.
         * Return nullptr if the drawable does not exist.
         */
        TrianglesDrawable *get_triangles_drawable(const std::string &name) const;

        /**
         * Create a new points drawable and add it to this renderer.
         * @param name The name of the points drawable to be created.
         * @return The created points drawable. If a points drawable with 'name' already exists, the creation will be
         *         ignored and the existing drawable is returned.
         */
        PointsDrawable *add_points_drawable(const std::string &name);

        /**
         * Create a new lines drawable and add it to this renderer.
         * @param name The name of the lines drawable to be created.
         * @return The created lines drawable. If a lines drawable with 'name' already exists, the creation will be
         *         ignored and the existing drawable is returned.
         */
        LinesDrawable *add_lines_drawable(const std::string &name);

        /**
         * Create a new triangles drawable and add it to this renderer.
         * @param name The name of the triangles drawable to be created.
         * @return The created triangles drawable. If a triangles drawable with 'name' already exists, the creation will
         *         be ignored and the existing drawable is returned.
         */
        TrianglesDrawable *add_triangles_drawable(const std::string &name);

        /**
         * All available points drawables managed by this renderer.
         */
        const std::vector<PointsDrawable *> &points_drawables() const { return points_drawables_; }

        /**
         * All available lines drawables managed by this renderer.
         */
        const std::vector<LinesDrawable *> &lines_drawables() const { return lines_drawables_; }

        /**
         * All available triangles drawables managed by this renderer.
         */
        const std::vector<TrianglesDrawable *> &triangles_drawables() const { return triangles_drawables_; }

    public:
        /**
         * @brief Create default drawables for rendering.
         * @details This method creates defaults drawables for rendering a model. The supported default drawables are
         *              - PointCloud: "vertices".
         *              - SurfaceMesh: "faces", "vertices", "edges", "borders".
         *              - Graph: "vertices", "edges".
         *          These drawables are usually sufficient for basic rendering of the model. In case
         *          the default drawables don't meet the particular visualization purpose, you can
         *          override this function or set 'create_default_drawables' to false and create the
         *          drawables by calling Model::add_[type]_drawable().
         */
        static void create_default_drawables(Model *model);

        // -------------------------------------------------------------------------------------------------------------

        /**
         * @brief Set the default rendering state of the "vertices" drawable of a point cloud.
         * @details The default rendering state is determined by the availability of the vertex properties.
         *          The motivation is that the most appealing rendering is demonstrated by default. The following
         *          priority applies:
         *              1. per-vertex color: in "v:color";
         *              2. per-vertex texture coordinates: in "v:texcoord";
         *              3. segmentation: in "v:primitive_index";
         *              4: uniform color.
         */
        static void set_default_rendering_state(PointCloud *model, PointsDrawable *drawable);

        /**
         * @brief Set the default rendering state of the "faces" drawable of a surface mesh.
         * @details The default rendering state is determined by the availability of the vertex/face properties.
         *          The motivation is that the most appealing rendering is demonstrated by default. The following
         *          priority applies:
         *              1: per-face color: in "f:color";
         *              2: per-vertex color: in "v:color";
         *              3. per-halfedge texture coordinates: in "h:texcoord";
         *              4. per-vertex texture coordinates: in "v:texcoord";
         *              5. segmentation: in "f:chart";
         *              6 uniform color
         */
        static void set_default_rendering_state(SurfaceMesh *model, TrianglesDrawable *drawable);

        // -------------------------------------------------------------------------------------------------------------

        /**
         * @brief Generate a color property for visualizing segmentation. Each segment will be given a random color.
         * @param model  The model.
         * @param segments  The name of the <int> type face property storing the segmentation.
         * @param color_name The name of the color property to be created.
         */
        static void color_from_segmentation(SurfaceMesh *model, const std::string &segments,
                                            const std::string &color_name = "f:color_segments");

        /**
         * @brief Generate a color property for visualizing segmentation. Each segment will be given a random color.
         * @param model  The model.
         * @param segments  The name of the <int> type vertex property storing the segmentation.
         * @param color_name The name of the color property to be created.
         */
        static void color_from_segmentation(PointCloud *model, const std::string &segments,
                                            const std::string &color_name = "v:segments");

    protected:
        Model *model_;

        bool visible_;
        bool selected_;

        std::vector<PointsDrawable *> points_drawables_;
        std::vector<LinesDrawable *> lines_drawables_;
        std::vector<TrianglesDrawable *> triangles_drawables_;
    };
}

#endif  // SCAN3D_RENDERER_RENDERER_H
