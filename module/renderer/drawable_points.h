#ifndef SCAN3D_RENDERER_DRAWABLE_POINTS_H
#define SCAN3D_RENDERER_DRAWABLE_POINTS_H

#include <module/renderer/drawable.h>


namespace scan3d {


    // The drawable for rendering a set of points, e.g., point clouds, vertices of a mesh
	class PointsDrawable : public Drawable {
	public:
        PointsDrawable(const std::string& name = "", Model* model = nullptr);

        Type type() const override;

        // a point imposter can be a sphere, or a surfel/disc
        enum ImposterType {
            PLAIN,
            SPHERE,
            SURFEL
        };

        /** Get/Set impostor type. */
        ImposterType impostor_type() const { return impostor_type_; }
        void set_impostor_type(ImposterType t) { impostor_type_ = t; }

        /** Get/Set point size. */
        float point_size() const { return point_size_; }
        void set_point_size(float s) { point_size_ = s; }

        // Rendering.
        virtual void draw(const Camera* camera, bool with_storage_buffer = false) const override;

    protected:
        // without texture
        void _draw_plain_points(const Camera* camera, bool with_storage_buffer) const;
        void _draw_spheres_sprite(const Camera* camera, bool with_storage_buffer) const;
        void _draw_spheres_geometry(const Camera* camera, bool with_storage_buffer) const;
        void _draw_surfels(const Camera* camera, bool with_storage_buffer) const;

        // textured
        void _draw_plain_points_with_texture(const Camera* camera, bool with_storage_buffer) const;
        void _draw_spheres_with_texture_sprite(const Camera* camera, bool with_storage_buffer) const;
        void _draw_spheres_with_texture_geometry(const Camera* camera, bool with_storage_buffer) const;
        void _draw_surfels_with_texture(const Camera* camera, bool with_storage_buffer) const;

	private:
        float           point_size_;
        ImposterType    impostor_type_;
	};

}


#endif  // SCAN3D_RENDERER_DRAWABLE_POINTS_H
