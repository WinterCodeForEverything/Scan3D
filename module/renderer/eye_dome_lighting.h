#ifndef SCAN3D_RENDERER_EYE_DOME_LIGHTING_H
#define SCAN3D_RENDERER_EYE_DOME_LIGHTING_H


#include <string>

namespace scan3d {
    /*
    * EyeDomeLighting implements an EDL offscreen shading method, which is usefull
    * for rendering point clouds without normal information.
    *
    * Two image resolutions are used.
    *
    * This pass expects an initialized depth buffer and color buffer.
    * Initialized buffers means they have been cleared with farest z-value and
    * background color/gradient/transparent color.
    * An opaque pass may have been performed right after the initialization.
    *
    * ptimization tips: rendering with multi-effects (e.g., shadowing, SSAO)
    * an benefit from sharing the same geometry pass.
    *
    * Example usage:
    * 		if (camera()->type() == Camera::PERSPECTIVE)
    *			edl_->begin(width(), height(), camera()->zNear(), camera()->zFar());
    *		else {
    *			float r = camera()->sceneRadius();
    *			edl_->begin(width(), height(), 2.0f * r, 10.0f * r);
    *		}
    *      draw(); // your rendering code here
    *      edl_->end(camera()->sceneRadius());
    */

    class Camera;
    class ShaderProgram;
    class FramebufferObject;

    class EyeDomeLighting
    {
    public:
        EyeDomeLighting(Camera* cam);
        virtual ~EyeDomeLighting();

        void begin();
        void end();

    protected:

        // Initialization of required framebuffer objects
        void init_frame_buffers(int w, int h);

        // Initialization of required GLSL shaders
        void init_shader_programs();

        // Render EDL in full resolution buffer
        void shade_high(float scene_radius);

        // Render EDL in middle resolution buffer
        void shade_low();

        // Render EDL in middle resolution buffer
        void blur_low();

        // Compose color and shaded images
        void compose();

        // Release graphics resources
        void clear();

    private:
        Camera* camera_;

        // Framebuffer object for initial projection

        // used to record scene data
        FramebufferObject*	projection_fbo_;

        //////////////////////////////////////////////////////////////////////////

        // Framebuffer objects for EDL

        // for EDL full res shading
        FramebufferObject*	high_fbo_;

        // for EDL low res shading (image size/4)
        FramebufferObject*	low_fbo_;

        //////////////////////////////////////////////////////////////////////////

        // Shader programs
        ShaderProgram* shade_program_;
        ShaderProgram* compose_program_;
        ShaderProgram* blur_program_;

        float neighbours_[8][4];
        int   low_res_factor_; // basically 4

        //////////////////////////////////////////////////////////////////////////

        float z_near_;  // near clipping plane
        float z_far_;	// far clipping plane

        int width_;
        int height_;

    private:
        //copying disabled
        EyeDomeLighting(const EyeDomeLighting&);
        EyeDomeLighting& operator=(const EyeDomeLighting&);
    };

}

#endif	// SCAN3D_RENDERER_EYE_DOME_LIGHTING_H
