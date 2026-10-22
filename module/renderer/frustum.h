#ifndef SCAN3D_RENDERER_FRUSTUM_H
#define SCAN3D_RENDERER_FRUSTUM_H


#include <vector>

#include <module/core/types.h>


namespace scan3d {

    class Frustum
    {
    public:
        enum ProjectionType { PERSPECTIVE, ORTHO };

    public:
        Frustum(ProjectionType type);

        // To define a working frustum, you need to call two functions:
        //   - orient(), defining the view matrix.
        //   - set_perspective()/set_frustum(), or set_ortho(), defining the projection matrix.
        void orient(const vec3 &pos, const vec3& at, const vec3& up);

        // NOTE: fovy is in radians
        void set_perspective(float fovy, float aspect, float znear, float zfar);
        void set_frustum(float frustum_width, float frustum_height, float znear, float zfar);

        void set_ortho(float xmin, float xmax, float ymin, float ymax, float znear, float zfar);

        mat4 view_matrix() const;
        mat4 projection_matrix() const;

        // the center point of the near/far planes in world space
        vec3  near_center() const;
        vec3  far_center() const;

        float near_width() const;	// frustum width at near plane
        float near_height() const;	// frustum height at near plane

        float far_width() const;	// frustum width at far plane
        float far_height() const;	// frustum height at far plane

        // in world space
        const vec3& position() const { return pos_; }
        // the up/right vector in world space
        vec3 up_vector() const;
        vec3 right_vector() const;

        float near_distance() const { return near_; }
        float far_distance() const { return far_; }

        // compute the compute the 8 corner points in world space
        std::vector<vec3> vertices() const;

    private:
        ProjectionType type_;

        vec3 pos_;
        vec3 at_;
        vec3 up_;

        float near_, far_;
        float xmin_, xmax_, ymin_, ymax_;
        float fovy_, ar_;
    };

}


#endif // SCAN3D_RENDERER_FRUSTUM_H
