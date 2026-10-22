#include <tools/tool_selection_surface_mesh.h>
#include <tools/tool_manager.h>
#include <tools/canvas.h>

#include <module/gui/picker_model.h>
#include <module/gui/picker_surface_mesh.h>
#include <module/renderer/drawable_triangles.h>
#include <module/renderer/renderer.h>
#include <module/util/logging.h>


namespace scan3d {

    namespace tools {

        // -------------------- ToolSurfaceMeshFaceSelection ----------------------

        ToolSurfaceMeshFaceSelection::ToolSurfaceMeshFaceSelection(ToolManager *mgr)
                : Tool(mgr) {
        }


        // -------------------- Click Select ----------------------

        ToolSurfaceMeshFaceSelectionClick::ToolSurfaceMeshFaceSelectionClick(ToolManager *mgr, SurfaceMeshPicker *picker, SelectMode mode)
                : ToolSurfaceMeshFaceSelection(mgr)
                , picker_(picker)
                , select_mode_(mode)
        {
            model_picker_ = new ModelPicker(mgr->viewer()->camera());
        }


        ToolSurfaceMeshFaceSelectionClick::~ToolSurfaceMeshFaceSelectionClick() {
            if (model_picker_)
                delete model_picker_;
        }


        void ToolSurfaceMeshFaceSelectionClick::press(int x, int y) {
            SurfaceMesh::Face picked_face = SurfaceMesh::Face();

            SurfaceMesh *mesh = multiple_pick(picked_face, x, y);
            if (mesh && picked_face.is_valid()) {
                std::cout << "face selected: " << picked_face << std::endl;
//                MeshFacetAttribute<bool> status(mesh, select_attr_name);
//                status[picked_facet] = (select_mode_ == SM_SELECT);
//
//                if (select_mode_ == SM_SELECT)
//                    Logger::out(title()) << "1 facet selected" << std::endl;
//                else
//                    Logger::out(title()) << "1 facet deselected" << std::endl;
//
//                int id = picker_->picked_facet_id();
//                dynamic_cast<FacesDrawable *>(mesh->drawable(DT_FACES, "surface"))->set_selected(id,
//                                                                                                 select_mode_ ==
//                                                                                                 SM_SELECT);
//
//                mesh->notify_selection_change();
            }
        }


        SurfaceMesh *ToolSurfaceMeshFaceSelectionClick::multiple_pick(SurfaceMesh::Face& face, int x, int y) {
            face = SurfaceMesh::Face();

            Model *model = model_picker_->pick(tool_manager()->viewer()->models(), x, y);
            SurfaceMesh *picked_mesh = dynamic_cast<SurfaceMesh *>(model);
            if (!picked_mesh)
                return nullptr;

            face = picker_->pick_face(picked_mesh, x, y);
            return picked_mesh;
        }


        MultitoolSurfaceMeshFaceSelectionClick::MultitoolSurfaceMeshFaceSelectionClick(ToolManager *mgr)
                : MultiTool(mgr)
        {
            picker_ = new SurfaceMeshPicker(mgr->viewer()->camera());
            set_tool(LEFT_BUTTON, new ToolSurfaceMeshFaceSelectionClick(mgr, picker_, SM_SELECT));
            set_tool(RIGHT_BUTTON, new ToolSurfaceMeshFaceSelectionClick(mgr, picker_, SM_DESELECT));
        }


        MultitoolSurfaceMeshFaceSelectionClick::~MultitoolSurfaceMeshFaceSelectionClick() {
            delete picker_;
        }


        void MultitoolSurfaceMeshFaceSelectionClick::prepare_hint(ToolButton button, int x, int y) {
            clear_hint();
            if (button == NO_BUTTON && picker_) {
                SurfaceMesh::Face face;
                auto *mesh = dynamic_cast<ToolSurfaceMeshFaceSelectionClick *>(get_tool(LEFT_BUTTON))->multiple_pick(face, x, y);
                if (mesh && face.is_valid()) {
                    auto triangle_range = mesh->face_property<std::pair<int, int> >("f:triangle_range");
                    auto drawable = mesh->renderer()->get_triangles_drawable("faces");
                    drawable->set_highlight(true);
                    drawable->set_highlight_range(triangle_range[face]);
                }
            }
        }

        void MultitoolSurfaceMeshFaceSelectionClick::clear_hint() {
            for (auto model : tool_manager()->viewer()->models()) {
                SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(model);
                if (mesh) {
                    auto drawable = mesh->renderer()->get_triangles_drawable("faces");
                    drawable->set_highlight(false);
                    drawable->set_highlight_range({-1, -1});
                }
            }
        }


        // -------------------- Rect Select ----------------------

        ToolSurfaceMeshFaceSelectionRect::ToolSurfaceMeshFaceSelectionRect(ToolManager *mgr, SurfaceMeshPicker *picker, SelectMode mode)
                : ToolSurfaceMeshFaceSelection(mgr), picker_(picker), select_mode_(mode) {
        }


        void ToolSurfaceMeshFaceSelectionRect::press(int x, int y) {
            start_ = vec2(x, y);
        }


        void ToolSurfaceMeshFaceSelectionRect::drag(int x, int y) {
        }


        void ToolSurfaceMeshFaceSelectionRect::release(int x, int y) {
            int count = 0;
            for (auto model : tool_manager()->viewer()->models()) {
                SurfaceMesh *mesh = dynamic_cast<SurfaceMesh*>(model);
                int num = picker_->pick_faces(mesh, Rect(start_, vec2(x, y)), select_mode_ == SM_DESELECT);
                std::cout << num << " faces marked" << std::endl;

                count += num;

//                MeshFacetAttribute<bool> status(mesh, select_attr_name);
//                for (std::size_t i = 0; i < facets.size(); ++i) {
//                    MeshTypes::Facet *f = facets[i];
//                    if (status[f] != (select_mode_ == SM_SELECT)) {
//                        status[f] = (select_mode_ == SM_SELECT);
//                        ++count;
//                    }
//                }
//
//                // mark in shader storage buffer
//                MeshFacetAttribute<int> faceID(mesh, "faceID");
//                //#pragma omp parallel for // this will NOT work due to the bitwise operations
//                for (int i = 0; i < (int) facets.size(); ++i) {
//                    unsigned int id = faceID[facets[i]];
//                    dynamic_cast<FacesDrawable *>(mesh->drawable(DT_FACES, "surface"))->set_selected(id, select_mode_ ==
//                                                                                                         SM_SELECT);
//                }
//                mesh->notify_selection_change();
            }

            if (select_mode_ == SM_SELECT)
                LOG(INFO) << count << " facet selected.";
            else
                LOG(INFO) << count << " facet deselected.";
        }


        MultitoolSurfaceMeshFaceSelectionRect::MultitoolSurfaceMeshFaceSelectionRect(ToolManager *mgr)
                : MultiTool(mgr) {
            picker_ = new SurfaceMeshPicker(mgr->viewer()->camera());
            set_tool(LEFT_BUTTON, new ToolSurfaceMeshFaceSelectionRect(mgr, picker_, SM_SELECT));
            set_tool(RIGHT_BUTTON, new ToolSurfaceMeshFaceSelectionRect(mgr, picker_, SM_DESELECT));

            clear_hint();
        }


        MultitoolSurfaceMeshFaceSelectionRect::~MultitoolSurfaceMeshFaceSelectionRect() {
            delete picker_;
        }


        void MultitoolSurfaceMeshFaceSelectionRect::press(ToolButton button, int x, int y) {
            MultiTool::press(button, x, y);
            start_ = vec2(x, y);
            end_ = vec2(x, y);
        }


        void MultitoolSurfaceMeshFaceSelectionRect::prepare_hint(ToolButton button, int x, int y) {
            if (button != NO_BUTTON && picker_) {
                end_ = vec2(x, y);
            }
        }

        void MultitoolSurfaceMeshFaceSelectionRect::clear_hint() {
            start_ = vec2(0, 0);
            end_ = vec2(0, 0);
        }

        void MultitoolSurfaceMeshFaceSelectionRect::draw_hint() const {
            draw_rect(Rect(start_, end_));
        }

        // ------------------ Lasso Select -----------------------

        ToolSurfaceMeshFaceSelectionLasso::ToolSurfaceMeshFaceSelectionLasso(ToolManager *mgr, SurfaceMeshPicker *picker,
                                                           SelectMode mode)
                : ToolSurfaceMeshFaceSelection(mgr), picker_(picker), select_mode_(mode) {
        }

        void ToolSurfaceMeshFaceSelectionLasso::press(int x, int y) {
            lasso_.clear();
            lasso_.push_back(vec2(float(x), float(y)));
        }


        void ToolSurfaceMeshFaceSelectionLasso::drag(int x, int y) {
            lasso_.push_back(vec2(float(x), float(y)));
        }


        void ToolSurfaceMeshFaceSelectionLasso::release(int x, int y) {
            if (lasso_.size() < 3)
                return;

            int count = 0;
            for (auto model : tool_manager()->viewer()->models()) {
                SurfaceMesh *mesh = dynamic_cast<SurfaceMesh*>(model);
                int num = picker_->pick_faces(mesh, lasso_, select_mode_ == SM_DESELECT);
                std::cout << num << " faces marked" << std::endl;

                count += num;

//                MeshFacetAttribute<bool> status(mesh, select_attr_name);
//                for (std::size_t i = 0; i < facets.size(); ++i) {
//                    MeshTypes::Facet *f = facets[i];
//                    if (status[f] != (select_mode_ == SM_SELECT)) {
//                        status[f] = (select_mode_ == SM_SELECT);
//                        ++count;
//                    }
//                }
//
//                // mark in shader storage buffer
//                MeshFacetAttribute<int> faceID(mesh, "faceID");
//                //#pragma omp parallel for // this will NOT work due to the bitwise operations
//                for (int i = 0; i < (int) facets.size(); ++i) {
//                    unsigned int id = faceID[facets[i]];
//                    dynamic_cast<FacesDrawable *>(mesh->drawable(DT_FACES, "surface"))->set_selected(id, select_mode_ ==
//                                                                                                         SM_SELECT);
//                }
//                mesh->notify_selection_change();
            }

            if (select_mode_ == SM_SELECT)
                LOG(INFO) << count << " facet selected.";
            else
                LOG(INFO) << count << " facet deselected.";

            lasso_.clear();
        }


        MultitoolSurfaceMeshFaceSelectionLasso::MultitoolSurfaceMeshFaceSelectionLasso(ToolManager *mgr)
                : MultiTool(mgr) {
            picker_ = new SurfaceMeshPicker(mgr->viewer()->camera());
            set_tool(LEFT_BUTTON, new ToolSurfaceMeshFaceSelectionLasso(mgr, picker_, SM_SELECT));
            set_tool(RIGHT_BUTTON, new ToolSurfaceMeshFaceSelectionLasso(mgr, picker_, SM_DESELECT));

            clear_hint();
        }


        MultitoolSurfaceMeshFaceSelectionLasso::~MultitoolSurfaceMeshFaceSelectionLasso() {
            delete picker_;
        }


        void MultitoolSurfaceMeshFaceSelectionLasso::press(ToolButton button, int x, int y) {
            MultiTool::press(button, x, y);
            lasso_.clear();
            lasso_.push_back(vec2(float(x), float(y)));
        }


        void MultitoolSurfaceMeshFaceSelectionLasso::prepare_hint(ToolButton button, int x, int y) {
            if (button != NO_BUTTON)
                lasso_.push_back(vec2(float(x), float(y)));
        }

        void MultitoolSurfaceMeshFaceSelectionLasso::clear_hint() {
            lasso_.clear();
        }

        void MultitoolSurfaceMeshFaceSelectionLasso::draw_hint() const {
            draw_lasso(lasso_);
        }

    }

}