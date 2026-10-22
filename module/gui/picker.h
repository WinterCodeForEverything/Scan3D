#ifndef SCAN3D_GUI_PICKER_H
#define SCAN3D_GUI_PICKER_H

#include <module/core/types.h>
#include <module/renderer/camera.h>


namespace scan3d {

    class FramebufferObject;

    class Picker {
    public:
        Picker(const Camera *cam);

        ~Picker();

        const Camera *camera() const { return camera_; }

        /**
         * Construct a picking line.
         * @param x The cursor x-coordinate, relative to the left edge of the content area.
         * @param y The cursor y-coordinate, relative to the top edge of the content area.
         * @attention The screen point is expressed in the screen coordinate system with an origin in the upper left
         *            corner. So it doesn't necessarily correspond to a pixel on High DPI devices, e.g. a Mac with
         *            a Retina display.
         */
        Line3 picking_line(int x, int y) const {
            const vec3 p_near = unproject(x, y, 0);
            const vec3 p_far = unproject(x, y, 1);
            return Line3::from_two_points(p_near, p_far);
        }

        /**
         * The picking direction, pointing inside the screen.
         * @param x The cursor x-coordinate, relative to the left edge of the content area.
         * @param y The cursor y-coordinate, relative to the top edge of the content area.
         * @attention The screen point is expressed in the screen coordinate system with an origin in the upper left
         *            corner. So it doesn't necessarily correspond to a pixel on High DPI devices, e.g. a Mac with
         *            a Retina display.
         */
        vec3 picking_dir(int x, int y) const {
            return picking_line(x, y).direction();
        }

        /**
         * Project a 3D point in the world coordinate system onto the 2D screen coordinate system.
         * @param p A 3D point in the world coordinate system.
         * @return The x and y components of the returned value denote the projected screen point expressed in the
         *         screen coordinate system, with (0, 0) being the upper left corner of the content area. The z
         *         component ranges between 0.0 (near plane) and 1.0 (excluded, far plane).
         * @attention The screen point is expressed in the screen coordinate system with an origin in the upper left
         *            corner. So it doesn't necessarily correspond to a pixel on High DPI devices, e.g. a Mac with
         *            a Retina display.
         */
        // 3D space to screen
        vec3 project(const vec3 &p) const {
            return camera()->projectedCoordinatesOf(p);
        }

        /**
         * Compute the world coordinates of a point defined in the screen coordinate system.
         * @param x The cursor x-coordinate, relative to the left edge of the content area.
         * @param y The cursor y-coordinate, relative to the top edge of the content area.
         * @param depth The depth value of the screen point, ranging between 0.0 and 1.0 (excluded).
         * @return The world unprojected coordinates of the screen point.
         * @attention The screen point is expressed in the screen coordinate system with an origin in the upper left
         *            corner. So it doesn't necessarily correspond to a pixel on High DPI devices, e.g. a Mac with
         *            a Retina display.
         */
        vec3 unproject(int x, int y, float depth) const {
            return camera()->unprojectedCoordinatesOf(vec3(static_cast<float>(x), static_cast<float>(y), depth));
        }

        /**
         * Convert a point expressed in the screen coordinate system (with an origin in the upper left corner) into the
         * OpenGL coordinate system (with an origin in the lower left corner). The high DPI scaling is also taken into
         * consideration, so the result always corresponds to its image pixel.
         * @param x The x-coordinate, relative to the left edge of the content area.
         * @param y The y-coordinate, relative to the top edge of the content area.
         * @param gl_x Returns the x component of the point in the OpenGL coordinate system.
         * @param gl_y Returns the y component of the point in the OpenGL coordinate system.
         * @param width The width of the OpenGL viewport (may not be identical to the width of the screen in pixels).
         * @param height The width of the OpenGL viewport (may not be identical to the height of the screen in pixels).
         */
        void screen_to_opengl(int x, int y, int &gl_x, int &gl_y, int width, int height) const;

    protected:

        // prepare a frame buffer for the offscreen rendering
        void setup_framebuffer(int width, int height);

    protected:
        const Camera *camera_;

        bool use_gpu_if_supported_; // use GPU if supported

        // All the picking tasks can share the same framebuffer
        static FramebufferObject *fbo_;
    };

}

#endif  // SCAN3D_GUI_PICKER_H
