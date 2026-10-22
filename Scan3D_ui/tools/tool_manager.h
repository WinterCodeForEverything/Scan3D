#ifndef SCAN3D_GUI_TOOL_MANAGER_H
#define SCAN3D_GUI_TOOL_MANAGER_H

#include <tools/tool.h>

#include <map>


namespace scan3d {

    class Canvas;

    namespace tools {

        class MultiTool;

        class ToolManager {
        public:
            enum ToolName {
                //------------------- Surface Mesh Face Selection Tools ---------------------
                SELECT_SURFACE_MESH_FACE_CLICK_TOOL,
                SELECT_SURFACE_MESH_FACE_RECT_TOOL,
                SELECT_SURFACE_MESH_FACE_LASSO_TOOL,

                //------------------- Point Cloud Vertices Select Tools ---------------------
                SELECT_POINT_CLOUD_LASSO_TOOL,
                SELECT_POINT_CLOUD_RECT_TOOL,

                //------------------------------ empty tool ---------------------------------
                EMPTY_TOOL
            };

        public:
            ToolManager(Canvas *cvs);
            virtual ~ToolManager();

            void set_tool(ToolName name);

            MultiTool *current_tool() { return current_tool_; }
            ToolName current_tool_name() const { return current_tool_name_; }

            void press(ToolButton button, int x, int y);
            void drag(ToolButton button, int x, int y);
            void release(ToolButton button, int x, int y);

            // witch between tool function and scene manipulation
            void switch_mode();

            void draw_hint() const;
            void clear_hint();

            Canvas* viewer() const { return viewer_; }

        protected:
            virtual MultiTool *create_new_tool(ToolName name);

        private:
            Canvas *viewer_;

            MultiTool *current_tool_;
            ToolName current_tool_name_;

            MultiTool *last_tool_;
            ToolName last_tool_name_;

            typedef std::map<ToolName, MultiTool *> ToolMap;
            ToolMap tools_;
        };

    }

}


#endif  // SCAN3D_GUI_TOOL_MANAGER_H
