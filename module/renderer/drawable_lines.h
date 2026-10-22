#ifndef SCAN3D_RENDERER_DRAWABLE_LINES_H
#define SCAN3D_RENDERER_DRAWABLE_LINES_H

#include <module/renderer/drawable.h>


namespace scan3d {

    // The drawable for rendering a set of line segments, e.g., edges of a mesh, vector fields
	class LinesDrawable : public Drawable {
	public:
        LinesDrawable(const std::string& name = "", Model* model = nullptr);

        Type type() const override;

        // a line poster can be a cylinder, or a cone
        enum ImposterType {
            PLAIN,
            CYLINDER,
            CONE
        };
        ImposterType impostor_type() const { return impostor_type_; }
        void set_impostor_type(ImposterType t) { impostor_type_ = t; }

        float line_width() const { return line_width_; }
        void set_line_width(float w) { line_width_ = w; }

        // Rendering.
        virtual void draw(const Camera* camera, bool with_storage_buffer = false) const override;

    protected:
        // without texture
        void _draw_plain_lines(const Camera* camera, bool with_storage_buffer) const;
        void _draw_cylinders(const Camera* camera, bool with_storage_buffer) const;
        void _draw_cones(const Camera* camera, bool with_storage_buffer) const;

        // textured
        void _draw_plain_lines_with_texture(const Camera* camera, bool with_storage_buffer) const;
        void _draw_cylinders_with_texture(const Camera* camera, bool with_storage_buffer) const;
        void _draw_cones_with_texture(const Camera* camera, bool with_storage_buffer) const;

    private:
        float           line_width_;
        ImposterType    impostor_type_;
    };

}


#endif  // SCAN3D_RENDERER_DRAWABLE_LINES_H
