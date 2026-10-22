#ifndef SCAN3D_RENDERER_DRAWABLE_TRIANGLES_H
#define SCAN3D_RENDERER_DRAWABLE_TRIANGLES_H

#include <module/renderer/drawable.h>


namespace scan3d {


    // The drawable for rendering a set of triangles, e.g., the surface of a triangular mesh.
    // NOTE: it supports triangles only. To visualize general polygons, the vertex coordinates
    //       and properties (e.g., color, normal) should be provided as consecutive triplets
    //       in an array to be transferred to GPU. See update_vertex_buffer().
    class TrianglesDrawable : public Drawable {
	public:
        TrianglesDrawable(const std::string& name = "", Model* model = nullptr);

        Type type() const override;

        bool smooth_shading() const { return smooth_shading_; }
        void set_smooth_shading(bool b) { smooth_shading_ = b; }

        /**
         * @brief Query the opacity of the drawable, in the range [0.0, 1.0].
         * @return The opacity of the drawable.
         */
        float opacity() const { return opacity_; }

        /**
         * @brief Set the opacity of the drawable.
         * @param opacity The new opacity value, in the range [0.0, 1.0].
         */
        void set_opacity(float opacity) { opacity_ = opacity; }

        // Rendering.
        virtual void draw(const Camera* camera, bool with_storage_buffer = false) const override;

    protected:
        // without texture
        void _draw_triangles(const Camera* camera, bool with_storage_buffer) const;

        // textured
        void _draw_triangles_with_texture(const Camera* camera, bool with_storage_buffer) const;

	private:
        bool    smooth_shading_;
        float   opacity_;
	};

}


#endif  // SCAN3D_RENDERER_DRAWABLE_TRIANGLES_H
