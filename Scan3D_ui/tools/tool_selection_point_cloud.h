#ifndef SCAN3D_GUI_TOOL_SELECTION_POINT_CLOUD_H
#define SCAN3D_GUI_TOOL_SELECTION_POINT_CLOUD_H

#include <tools/tool.h>
#include <module/core/types.h>


namespace scan3d {

    class PointCloud;
    class PointCloudPicker;
    
    namespace tools {

        class ToolManager;

        class ToolPointCloudSelection : public Tool {
        public:
            ToolPointCloudSelection(ToolManager *mgr);
            virtual ~ToolPointCloudSelection() {}
            std::string name() const { return "PointCloudSelect"; }
        };
        

        // -------------------- Rect Select ----------------------

        class ToolPointCloudSelectionRect : public ToolPointCloudSelection {
        public:
            ToolPointCloudSelectionRect(ToolManager *mgr, PointCloudPicker *picker, SelectMode mode = SM_SELECT);
            virtual void press(int x, int y);
            virtual void drag(int x, int y);
            virtual void release(int x, int y);

        protected:
            PointCloudPicker *picker_;
            SelectMode select_mode_;
            vec2 start_;
        };

        // -------------------- Lasso Select ----------------------

        class ToolPointCloudSelectionLasso : public ToolPointCloudSelection {
        public:
            ToolPointCloudSelectionLasso(ToolManager *mgr, PointCloudPicker *picker, SelectMode mode = SM_SELECT);
            virtual void press(int x, int y);
            virtual void drag(int x, int y);
            virtual void release(int x, int y);

        protected:
            PointCloudPicker *picker_;
            SelectMode select_mode_;
            Polygon2 lasso_;
        };


        //_____________________________________________________

        // (message, "btn1: select point; btn3: deselect point");
        class MultitoolPointCloudSelectionRect : public MultiTool {
        public:
            MultitoolPointCloudSelectionRect(ToolManager *mgr);
            ~MultitoolPointCloudSelectionRect();

            virtual void set_hint() {} // point is the only thing to show
            void press(ToolButton button, int x, int y);
            void prepare_hint(ToolButton button, int x, int y);
            void clear_hint();
            void draw_hint() const;

            std::string instruction() const {
                return "Left: select; Right: deselect";
            }

        protected:
            PointCloudPicker *picker_;
            vec2 start_, end_;
        };


        // (message, "btn1: select point; btn3: deselect point");
        class MultitoolPointCloudSelectionLasso : public MultiTool {
        public:
            MultitoolPointCloudSelectionLasso(ToolManager *mgr);
            ~MultitoolPointCloudSelectionLasso();

            virtual void set_hint() {} // point is the only thing to show

            void press(ToolButton button, int x, int y);
            void prepare_hint(ToolButton button, int x, int y);
            void clear_hint();
            void draw_hint() const;

            std::string instruction() const {
                return "Left: select; Right: deselect";
            }

        protected:
            PointCloudPicker *picker_;
            Polygon2 lasso_;
        };

    }

}

#endif  // SCAN3D_GUI_TOOL_SELECTION_POINT_CLOUD_H
