#ifndef SCAN3D_RENDERER_CLIPPING_PLANE_H
#define SCAN3D_RENDERER_CLIPPING_PLANE_H

#include <module/core/types.h>


namespace scan3d {

    class ManipulatedFrame;
    class Camera;
    class ShaderProgram;

    class ClippingPlane {
    public:
        ClippingPlane();
        virtual ~ClippingPlane();

        bool is_enabled() const { return enabled_; }
        void set_enabled(bool b) { enabled_ = b; }

        void fit_scene(const vec3 &center, float radius);

        vec3 center() const;
        vec3 normal() const;

        Plane3 plane0() const;    // clipping plane0
        Plane3 plane1() const;    // clipping plane1

        bool is_visible() const { return visible_; }
        void set_visible(bool v) { visible_ = v; }

        const vec4& color() const { return color_; }
        void set_color(const vec4& c) { color_ = c; }

        bool cross_section() const { return cross_section_; }
        void set_cross_section(bool b) { cross_section_ = b; }

        // relative to scene bounding box. Default value is 0.01;
        float cross_section_width() { return cross_section_width_; }
        void set_cross_section_width(float w) { cross_section_width_ = w; }

        void set_program(ShaderProgram *program);

        // arrow = true: also draw an arrow in the center of the plane.
        void draw(Camera* cam) const;

        virtual ManipulatedFrame *manipulated_frame();
        virtual const ManipulatedFrame *manipulated_frame() const;

    protected:
        ManipulatedFrame *manipulated_frame_;
        bool enabled_;
        bool visible_;
        vec4 color_;
        bool cross_section_;
        float cross_section_width_;
        float size_;
    };

}

#endif // SCAN3D_RENDERER_CLIPPING_PLANE_H
