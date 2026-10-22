#ifndef SCAN3D_RENDERER_TRANSPARENCY_H
#define SCAN3D_RENDERER_TRANSPARENCY_H

#include <vector>


namespace scan3d {

    class Camera;
    class TrianglesDrawable;

    // base class for rendering with transparency

    class Transparency
    {
    public:
        Transparency() {}
        virtual ~Transparency() {}

        virtual void draw(const std::vector<TrianglesDrawable*>& surfaces) = 0;
    };

}


#endif // SCAN3D_RENDERER_TRANSPARENCY_H
