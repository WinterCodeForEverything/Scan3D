#ifndef SCAN3D_RENDERER_OPENGL_TEXTURE_H
#define SCAN3D_RENDERER_OPENGL_TEXTURE_H


#include <string>
#include <vector>


namespace scan3d {

    // currently only TEXTURE_2D is supported
    class Texture {
    public:
        enum WrapMode {
            CLAMP_TO_EDGE, REPEAT
        };
        enum FilterMode {
            NEAREST, LINEAR
        };

        /**
         * Creates a texture from an image file.
         * @image_file The full path to the image file.
         * @return The created texture.
         */
        static Texture *
        create(const std::string &image_file, WrapMode wrap = CLAMP_TO_EDGE, FilterMode filter = LINEAR);

        /**
         * Creates a texture from the given image data.
         * @param rgb_data The image data.
         * @param width The width (i.e., number of columns) of the image.
         * @param height The rows (i.e., number of rows) of the image.
         * @param comp The number of components for each pixel (e.g., 3 for RGB)
         * @return The created texture.
         */
        static Texture *create(const std::vector<unsigned char> &rgb_data, int width, int height, int comp,
                               WrapMode wrap = CLAMP_TO_EDGE, FilterMode filter = LINEAR);

        ~Texture();

        unsigned int id() const { return id_; }
        const std::string &name() const { return name_; }

        void bind(int unit = 0);
        void release();

        int width() const { return sizes_[0]; }
        int height() const { return sizes_[1]; }

        int channels() const { return sizes_[2]; }

        WrapMode wrap_mode() const { return wrap_mode_; }
        FilterMode filter_mode() const { return filter_mode_; }

    private:
        unsigned int id_;
        int sizes_[3];

        std::string name_;
        WrapMode wrap_mode_;
        FilterMode filter_mode_;

    private:
        /** The creation of a texture is only allowed by using the create() function */
        Texture();

        //copying disabled
        Texture(const Texture &);
        Texture &operator=(const Texture &);

        friend class TextureManager;
    };


    /**
     * @brief Discretize a gradually varying-color (from left to right) image into a set of uniform colored vertical
     *        stripes.
     * @param data The input data to be discretized.
     * @param width The width of the image (i.e., number of pixels in a row).
     * @param height The height of the image (i.e., number of pixels in a column).
     * @param channels The number or color component per pixel.
     * @param num_stripes The number of stripes. The image will remain unmodified if num_stripes >= image width.
     */
    void discretize_image(
            std::vector<unsigned char> &data,
            int width,
            int height,
            int channels,
            int num_stripes
    );

} // namespace scan3d


#endif  // SCAN3D_RENDERER_OPENGL_TEXTURE_H

