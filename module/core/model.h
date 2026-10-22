#ifndef SCAN3D_CORE_MODEL_H
#define SCAN3D_CORE_MODEL_H


#include <string>
#include <vector>

#include <module/core/types.h>


namespace scan3d {

    class Renderer;

    /**
     * Model is the base class of renderable 3D models, e.g., SurfaceMesh, PointCloud, Graph.
     * A Model can have multiple drawables, e.g., faces, vertices, edges.
     */
    class Model
    {
    public:
        Model(const std::string& name = "unknown");
        virtual ~Model();

        const std::string& name() const { return name_; }
        void set_name(const std::string& n) { name_ = n; }

        /** The bounding box of the model. */
        const Box3& bounding_box() const;

        void update_bounding_box();

        /** The vertices of the model. */
        virtual const std::vector<vec3>& points() const = 0;

        /** Test if the model is empty. */
        bool empty() const { return points().empty(); };

        /** prints the names of all properties to an output stream (e.g., std::cout). */
        virtual void property_stats(std::ostream &output) const = 0;

        /** Set/Get the renderer of this model. */
        void set_renderer(Renderer* r) { renderer_ = r; }
        inline Renderer* renderer() { return renderer_; }
        inline const Renderer* renderer() const { return renderer_; }

    protected:
        std::string	name_;

        Box3		bbox_;
        bool		bbox_known_;

        Renderer* renderer_;
    };
}

#endif  // SCAN3D_CORE_MODEL_H
