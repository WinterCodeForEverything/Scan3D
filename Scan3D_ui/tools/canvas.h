#ifndef SCAN3D_GUI_CANVAS_H
#define SCAN3D_GUI_CANVAS_H

#include <vector>

#include "tools/tool_manager.h"

namespace scan3d {

    class Camera;
    class Model;

    namespace tools {
        class ToolManager;
    }

    // Canvas defined the basic interface for interactive tools
    class Canvas {
    public:
        Canvas() {
            tool_manager_ = new tools::ToolManager(this);
        }
        ~Canvas() { delete tool_manager_; }

        virtual Camera *camera() = 0;
        virtual const Camera *camera() const = 0;

        virtual const std::vector<Model*>& models() const = 0;

        tools::ToolManager* tool_manager() const { return tool_manager_; }

    private:
        tools::ToolManager* tool_manager_;
    };

}


#endif //SCAN3D_GUI_CANVAS_H
