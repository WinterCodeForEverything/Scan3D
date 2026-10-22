#include <tools/tool_manager.h>
#include <tools/tool_selection_point_cloud.h>
#include <tools/tool_selection_surface_mesh.h>

#include <module/util/logging.h>


namespace scan3d {

    namespace tools {

        ToolManager::ToolManager(Canvas* cvs)
                : viewer_(cvs)
                , current_tool_(nullptr)
                , current_tool_name_(EMPTY_TOOL)
                , last_tool_(nullptr)
                , last_tool_name_(EMPTY_TOOL)
        {
        }


        ToolManager::~ToolManager() {
            tools_.clear();
        }


        MultiTool *ToolManager::create_new_tool(ToolName name) {
            MultiTool *tool = nullptr;
            switch (name) {
                //------------------- Surface Mesh Face Selection Tools ---------------------
                case SELECT_SURFACE_MESH_FACE_CLICK_TOOL:
                    tool = new MultitoolSurfaceMeshFaceSelectionClick(this);
                    break;
                case SELECT_SURFACE_MESH_FACE_RECT_TOOL:
                    tool = new MultitoolSurfaceMeshFaceSelectionRect(this);
                    break;
                case SELECT_SURFACE_MESH_FACE_LASSO_TOOL:
                    tool = new MultitoolSurfaceMeshFaceSelectionLasso(this);
                    break;

                    //------------------- Point Cloud Vertices Select Tools ---------------------

                case SELECT_POINT_CLOUD_RECT_TOOL:
                    tool = new MultitoolPointCloudSelectionRect(this);
                    break;
                case SELECT_POINT_CLOUD_LASSO_TOOL:
                    tool = new MultitoolPointCloudSelectionLasso(this);
                    break;

                default:
                    break;
            }

            return tool;
        }


        void ToolManager::switch_mode() {
            if (current_tool_ == nullptr && last_tool_ != nullptr) {
                current_tool_ = last_tool_;
                current_tool_name_ = last_tool_name_;
                last_tool_ = nullptr;
                last_tool_name_ = EMPTY_TOOL;
                if (current_tool_)
                    current_tool_->set_hint();
            } else if (current_tool_ != nullptr) {
                last_tool_ = current_tool_;
                last_tool_name_ = current_tool_name_;
                last_tool_->clear_hint();
                current_tool_ = nullptr;
                current_tool_name_ = EMPTY_TOOL;
            }

            if (current_tool_)
                std::cout << current_tool_->instruction() << std::endl;


        }


        void ToolManager::set_tool(ToolName name) {
            ToolMap::iterator it = tools_.find(name);
            if (it == tools_.end() && name != EMPTY_TOOL) {
                MultiTool *tool = create_new_tool(name);
                if (tool)
                    tools_[name] = tool;
            }

            it = tools_.find(name);
            if (it == tools_.end()) {
                last_tool_ = current_tool_;
                last_tool_name_ = current_tool_name_;
                current_tool_ = nullptr;
                current_tool_name_ = EMPTY_TOOL;
            } else {
                last_tool_ = current_tool_;
                last_tool_name_ = current_tool_name_;
                current_tool_ = it->second;
                current_tool_name_ = it->first;
                current_tool_->set_hint();
                std::cout << current_tool_->instruction() << std::endl;
            }

            if (last_tool_)
                last_tool_->clear_hint();
        }


        void ToolManager::press(ToolButton button, int x, int y) {
            if (current_tool_ != nullptr) {
                current_tool_->clear_hint();
                current_tool_->press(button, x, y);
            }
        }


        void ToolManager::release(ToolButton button, int x, int y) {
            if (current_tool_ != nullptr) {
                current_tool_->clear_hint();
                current_tool_->release(button, x, y);
            }
        }


        void ToolManager::drag(ToolButton button, int x, int y) {
            if (current_tool_ != nullptr) {
                current_tool_->drag(button, x, y);
                current_tool_->prepare_hint(button, x, y);
            }
        }


        void ToolManager::draw_hint() const {
            if (current_tool_)
                current_tool_->draw_hint();
        }


        void ToolManager::clear_hint() {
            if (current_tool_)
                current_tool_->clear_hint();
        }

    }

}
