#ifndef SCAN3D_RENDERER_DRAWABLE_H
#define SCAN3D_RENDERER_DRAWABLE_H

#include <string>
#include <vector>
#include <functional>

#include <module/core/types.h>
#include <module/renderer/state.h>

namespace scan3d {

    class Model;
    class Camera;
    class VertexArrayObject;

    /**
     * @brief The base class for drawable objects. A drawable represent a set of points, line segments, or triangles.
     * @details A Drawable is an abstraction for "something that can be drawn", e.g., a point point cloud, the surface
     *          of a mesh, the wireframe of a surface mesh.
     *          A drawable manages its rendering status and controls the upload of the data to the GPU.
     *          A drawable can live independently or be associated with a Model.
     * @related @class Renderer
     */

    class Drawable : public State {
    public:
        // three types of drawables
        enum Type {
            DT_POINTS = 0x0000,     // == GL_POINTS
            DT_LINES = 0x0001,      // == GL_LINES
            DT_TRIANGLES = 0x0004   // == GL_TRIANGLES
        };

    public:
        // a drawable can be stand-alone or attached to a model
        Drawable(const std::string &name = "unknown", Model *model = nullptr);
        ~Drawable();

        /// Returns the type of the drawable.
        virtual Type type() const = 0;

        const std::string &name() const { return name_; }
        void set_name(const std::string n) { name_ = n; }

        /// the model to which the drawable is attached to (can be NULL).
        Model *model() { return model_; }
        const Model *model() const { return model_; }
        void set_model(Model *m) { model_ = m; }

        const Box3 &bounding_box() const;

        State& state() { return *this; };
        const State& state() const { return *this; };
        void set_state(const State& s) { state() = s; };

        /// print statistics (e.g., num vertices, memory usage) of the buffers to an output stream (e.g., std::cout).
        void buffer_stats(std::ostream &output) const;

        // ------------------- buffer access and management ------------------------

        unsigned int vertex_buffer() const { return vertex_buffer_; }
        unsigned int color_buffer() const { return color_buffer_; }
        unsigned int normal_buffer() const { return normal_buffer_; }
        unsigned int texcoord_buffer() const { return texcoord_buffer_; }
        unsigned int element_buffer() const { return element_buffer_; }
        unsigned int storage_buffer() const { return storage_buffer_; }
        unsigned int selection_buffer() const { return selection_buffer_; }

        /**
         * Create/Update a single buffer.
         * Primitives like lines and triangles can be drawn with or without the element buffer.
         *  - With an element buffer: this can reduce the GPU memory consumption.
         *  - Without an element buffer: easier data transfer, but uses more GPU memory. In this case, vertices need to
         *    be in a correct order, like f1_v1, f1_v2, f1_v3, f2_v1, f2_v2, f2_v3... This requires the shared vertices
         *    be duplicated in the vertex buffer.
         */
        void update_vertex_buffer(const std::vector<vec3> &vertices);
        void update_color_buffer(const std::vector<vec3> &colors);
        void update_normal_buffer(const std::vector<vec3> &normals);
        void update_texcoord_buffer(const std::vector<vec2> &texcoords);
        void update_element_buffer(const std::vector<unsigned int> &elements);

        // entry must have 2 or 3 elements
        void update_element_buffer(const std::vector< std::vector<unsigned int> > &elements);

        /// selection buffer (internally based on a shader storage buffer)
        /// @param index: the index of the binding point.
        /// NOTE: the buffers should also be bound to this point in all shader code
        void update_selection_buffer(unsigned int index = 1);

        /// generic storage buffer
        /// @param index: the index of the binding point.
        /// NOTE: the buffers should also be bound to this point in all shader code
        void update_storage_buffer(const void *data, std::size_t datasize, unsigned int index = 0);

        /// Releases the element buffer if existing vertex data is sufficient (may require duplicating vertex data).
        void release_element_buffer();

        // ----------------- access data from the buffers -------------------

        void fetch_selection_buffer();

        // -------------------------- rendering -----------------------------

        /// The draw method.
        virtual void draw(const Camera *camera, bool with_storage_buffer = false) const = 0;

        /// The internal draw method of this drawable.
        /// NOTE: this functions should be called when your shader program is in use,
        ///		 i.e., between glUseProgram(id) and glUseProgram(0);
        void gl_draw(bool with_storage_buffer = false) const;

        /**
         * @brief Requests an update of the OpenGL buffers.
         * @details This function sets the status to trigger an update of the OpenGL buffers. The actual update does not
         *          occur immediately but is deferred to the rendering phase.
         * @attention This method works for standard drawables (no update function required) and non-standard drawable
         *            (update function required). Standard drawables include:
         *              - SurfaceMesh: "faces", "edges", "vertices", "borders", "locks";
         *              - PointCloud: "vertices";
         *              - Graph: "edges", "vertices".
         */
        void update();

        /**
         * @brief Setups how a drwable can update its OpenGL buffers. This function is required by only non-standard
         *        drawables for a special visualization purpose. Standard drawables can be automatically updated and
         *        do not require this function.
         * @related renderer::update_buffers(...), update_buffers().
         */
        void set_update_func(std::function<void(Model*, Drawable*)> func) { update_func_ = func; }

    protected:
        VertexArrayObject *vao() const { return vao_; }

        void internal_update_buffers();

        void clear();

    protected:
        std::string name_;
        Model *model_;
        Box3 bbox_;

        // vertex array object
        VertexArrayObject *vao_;

        std::size_t num_vertices_;
        std::size_t num_indices_;

        bool update_needed_;
        std::function<void(Model*, Drawable*)> update_func_;

        unsigned int vertex_buffer_;
        unsigned int color_buffer_;
        unsigned int normal_buffer_;
        unsigned int texcoord_buffer_;
        unsigned int element_buffer_;

        unsigned int storage_buffer_;
        std::size_t current_storage_buffer_size_;

        unsigned int selection_buffer_;  // used for selection.
        std::size_t current_selection_buffer_size_; // in case the object is modified
    };

}


#endif  // SCAN3D_RENDERER_DRAWABLE_H
