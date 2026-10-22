#ifndef SCAN3D_GUI_TOOL_SEGMENTATION_SURFACE_MESH_H
#define SCAN3D_GUI_TOOL_SEGMENTATION_SURFACE_MESH_H


#include <tools/tool.h>
#include <module/core/surface_mesh.h>


namespace scan3d {

    class SurfaceMesh;
    class SurfaceMeshPicker;

    namespace tools {

        class ToolManager;

        class ToolSurfaceMeshSegmentation : public Tool {
        public:
            ToolSurfaceMeshSegmentation(ToolManager *mgr, SurfaceMeshPicker *picker) : Tool(mgr), picker_(picker) {
            }
            virtual ~ToolSurfaceMeshSegmentation() {}

            virtual void press(int x, int y) {}
            virtual void drag(int x, int y) {}
            virtual void release(int x, int y) {}

        protected:
            SurfaceMeshPicker *picker_;
        };


        // -------------------- Merge two patches ----------------------
        class ToolSurfaceMeshSegmentationMerge : public ToolSurfaceMeshSegmentation {
        public:
            ToolSurfaceMeshSegmentationMerge(ToolManager *mgr, SurfaceMeshPicker *picker) : ToolSurfaceMeshSegmentation(mgr, picker) {}
        };

        // -------------------- Split a patch ----------------------

        class ToolSurfaceMeshSegmentationSplit : public ToolSurfaceMeshSegmentation {
        public:
            ToolSurfaceMeshSegmentationSplit(ToolManager *mgr, SurfaceMeshPicker *picker): ToolSurfaceMeshSegmentation(mgr, picker) {}
        };

        //_____________________________________________________

        // (message, "Left: merge patches; Right: split patch");
        class MultitoolSurfaceMeshFaceSegmentation : public MultiTool {
        public:
            MultitoolSurfaceMeshFaceSegmentation(ToolManager *mgr);
            ~MultitoolSurfaceMeshFaceSegmentation();

            void press(ToolButton button, int x, int y) {}
            void prepare_hint(ToolButton button, int x, int y);
            void clear_hint();
            void draw_hint() const {}

            std::string instruction() const {
                return "Left: merge patches; Right: split patch";
            }

        protected:
            SurfaceMeshPicker *picker_;
        };
    }
}

#endif  // SCAN3D_GUI_TOOL_SEGMENTATION_SURFACE_MESH_H
