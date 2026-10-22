#ifndef SCAN3D_RENDERER_AVERAGE_COLOR_BLENDING_H
#define SCAN3D_RENDERER_AVERAGE_COLOR_BLENDING_H

#include <vector>
#include <module/renderer/transparency.h>


namespace scan3d {

    class Camera;
    class TrianglesDrawable;
    class FramebufferObject;

    // Optimization tips: rendering with multi-effects (e.g., shadowing, SSAO)
    // can benefit from sharing the same geometry pass.

    class AverageColorBlending : public Transparency
    {
    public:
        AverageColorBlending(Camera* cam);
        ~AverageColorBlending() override;

        void draw(const std::vector<TrianglesDrawable*>& surfaces) override;

    protected:

        // Allocate and configure FBOs, shader programs.
        void init(int w, int h);

        // Render the opaque pass geometry.
        void geometry_pass(const std::vector<TrianglesDrawable*>& surfaces);

        // Release all FBOs, shader programs.
        void clear();

    private:
        Camera*	camera_;
        float   bkg_color_[4];

        FramebufferObject* fbo_;

    private:
        //copying disabled
        AverageColorBlending(const AverageColorBlending&);
        AverageColorBlending& operator=(const AverageColorBlending&);
    };

}

#endif // SCAN3D_RENDERER_AVERAGE_COLOR_BLENDING_H
