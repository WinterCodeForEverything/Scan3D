#ifndef SCAN3D_GUI_PICKER_MODEL_H
#define SCAN3D_GUI_PICKER_MODEL_H

#include <module/gui/picker.h>
#include <module/renderer/state.h>
#include <module/renderer/camera.h>

#include <vector>
#include <unordered_map>


namespace scan3d {

    class Model;
    class Drawable;

    class ModelPicker : public Picker {
    public:
        ModelPicker(const Camera *cam);
        ~ModelPicker();

        /**
         * Pick a model from a set of models given the cursor position in the screen coordinate system.
         * @param x The cursor x-coordinate, relative to the left edge of the content area.
         * @param y The cursor y-coordinate, relative to the top edge of the content area.
         * @attention The screen point is expressed in the screen coordinate system with an origin in the upper left
         *            corner. So it doesn't necessarily correspond to a pixel on High DPI devices, e.g. a Mac with
         *            a Retina display.
         * @return The picked model.
         */
        Model *pick(const std::vector<Model *> &models, int x, int y);

    private:

        // render each model of the scene with a unique color
        void draw(const std::vector<Model *> &models);
        // render the drawable with color
        void draw(Drawable *drawable, const vec4 &color);

    private:

        std::unordered_map<Drawable *, State> states_;

    };

}


#endif  // SCAN3D_GUI_PICKER_MODEL_H
