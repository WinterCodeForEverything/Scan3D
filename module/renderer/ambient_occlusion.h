#ifndef EASY_RENDERER_AMBIENT_OCCLUSION_H
#define EASY_RENDERER_AMBIENT_OCCLUSION_H


#include <string>
#include <vector>

#include <module/core/types.h>


namespace scan3d {

    class Model;
    class Camera;
    class FramebufferObject;
	
	// AmbientOcclusion implements the traditional Screen Space Ambient Occlusion technique. 
    //
    // Optimization tips: rendering with multi-effects (e.g., shadowing, SSAO)
    // can benefit from sharing the same geometry pass.


    class AmbientOcclusion
    {   
    public:
		AmbientOcclusion(Camera* cam);
        virtual ~AmbientOcclusion();

        // used to compute sample radius (in pixels [0, 4]?).
        float radius() const { return radius_; }
        void  set_radius(float r) { radius_ = r; }

        float bias() const { return bias_; }
        void  set_bias(float b) { bias_ = b; }

        // generate and return the ssao texture
        virtual unsigned int generate(const std::vector<Model*>& models);
        // return the ssao texture
        unsigned int ssao_texture() const;

    protected:
        void init(int w, int h);

        void geometry_pass(const std::vector<Model*>& models);
        void ssao_pass();
        void blur_pass();

        void generate_noise(int width, int height);

    protected:
        Camera* camera_;

        float	radius_;
        float	bias_;

        FramebufferObject*	geom_fbo_;
        FramebufferObject*	ssao_fbo_;

        std::vector<vec3> ssao_kernel_;
        unsigned int	  noise_texture_;

    private:
        //copying disabled
		AmbientOcclusion(const AmbientOcclusion&);
        AmbientOcclusion& operator=(const AmbientOcclusion&);
    };

} // namespace scan3d


#endif	//  EASY_RENDERER_AMBIENT_OCCLUSION_H
