#ifndef SCAN3D_CORE_RANDOM_H
#define SCAN3D_CORE_RANDOM_H

#include <random>

#include <module/core/types.h>


namespace scan3d {


    /* Random real in [0, 1]. */
    inline float random_float() {
        return float(std::rand()) / float(RAND_MAX);
    }

    /* Random real number in the range [min, max] */
    inline float random_float(float min, float max) {
        return min + random_float() * (max - min);
    }

    inline vec3 random_color(bool allow_dark = false) {
        static float min_rgb = 0.3f;
        min_rgb = allow_dark ? 0.0f : min_rgb;
        return vec3(
            random_float(min_rgb, 1.0f),
            random_float(min_rgb, 1.0f),
            random_float(min_rgb, 1.0f)
        );
    }

}


#endif // SCAN3D_CORE_RANDOM_H
