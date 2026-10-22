#ifndef SCAN3D_GUI_PICKER_POINT_CLOUD_H
#define SCAN3D_GUI_PICKER_POINT_CLOUD_H


#include <module/gui/picker.h>


namespace scan3d {

    class PointCloud;
    class ShaderProgram;

    class PointCloudPicker : public Picker {
    public:
        PointCloudPicker(const Camera *cam);

        /**
         * @brief Pick vertices of a point cloud by a rectangle. The selected vertices will be marked in vertex property
         * "v:select".
         * @param rect The rectangle region.
         * @param deselect True to perform an inverse operation.
         * @return The number of vertices selected during this operation (despite their previous status).
         */
        int pick_vertices(PointCloud *model, const Rect& rect, bool deselect);

        /**
         * @brief Pick vertices of a point cloud by a polygon/lasso. The selected vertices will be marked in vertex
         * property "v:select".
         * @param plg The polygon region.
         * @param deselect True to perform an inverse operation.
         * @return The number of vertices selected during this operation (despite their previous status).
         */
        int pick_vertices(PointCloud *model, const Polygon2 &plg, bool deselect);

    private:
        int pick_vertices_cpu(PointCloud *model, const Rect& rect, bool deselect);
        int pick_vertices_gpu(PointCloud *model, const Rect& rect, bool deselect, ShaderProgram *program);

        int pick_vertices_cpu(PointCloud *model, const Polygon2 &plg, bool deselect);
        int pick_vertices_gpu(PointCloud *model, const Polygon2 &plg, bool deselect, ShaderProgram *program);
    };

}

#endif // SCAN3D_GUI_PICKER_POINT_CLOUD_H
