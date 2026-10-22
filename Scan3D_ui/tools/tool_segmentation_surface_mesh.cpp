#include <tools/tool_segmentation_surface_mesh.h>
#include <tools/tool_manager.h>
#include <tools/canvas.h>

#include <module/gui/picker_surface_mesh.h>
#include <module/renderer/drawable_triangles.h>
#include <module/renderer/renderer.h>
#include <module/util/logging.h>


namespace scan3d {

    namespace tools {

        MultitoolSurfaceMeshFaceSegmentation::MultitoolSurfaceMeshFaceSegmentation(ToolManager *mgr)
                : MultiTool(mgr)
        {
            picker_ = new SurfaceMeshPicker(mgr->viewer()->camera());
            set_tool(LEFT_BUTTON, new ToolSurfaceMeshSegmentationMerge(mgr, picker_));
            set_tool(RIGHT_BUTTON, new ToolSurfaceMeshSegmentationSplit(mgr, picker_));
        }


        MultitoolSurfaceMeshFaceSegmentation::~MultitoolSurfaceMeshFaceSegmentation() {
            delete picker_;
        }


        void MultitoolSurfaceMeshFaceSegmentation::prepare_hint(ToolButton button, int x, int y) {
//            clear_hint();
//            if (button == NO_BUTTON && picker_) {
//                SurfaceMesh::Face face;
//                auto *mesh = dynamic_cast<ToolSurfaceMeshFaceSelectionClick *>(get_tool(LEFT_BUTTON))->multiple_pick(face, x, y);
//                if (mesh && face.is_valid()) {
//                    auto triangle_range = mesh->face_property<std::pair<int, int> >("f:triangle_range");
//                    auto drawable = mesh->renderer()->get_triangles_drawable("faces");
//                    drawable->set_highlight(true);
//                    drawable->set_highlight_range(triangle_range[face]);
//                }
//            }
        }

        void MultitoolSurfaceMeshFaceSegmentation::clear_hint() {
            for (auto model : tool_manager()->viewer()->models()) {
                SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(model);
                if (mesh) {
                    auto drawable = mesh->renderer()->get_triangles_drawable("faces");
                    drawable->set_highlight(false);
                    drawable->set_highlight_range({-1, -1});
                }
            }
        }

    }

}