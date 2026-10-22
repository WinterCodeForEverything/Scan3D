#include <module/renderer/texture_manager.h>
#include <module/renderer/opengl.h>
#include <module/renderer/opengl_error.h>
#include <module/fileio/image_io.h>
#include <module/util/file_system.h>
#include <module/util/logging.h>

#include <iostream>


namespace scan3d {

    Texture::Texture()
            : id_(0), name_(""), wrap_mode_(CLAMP_TO_EDGE), filter_mode_(LINEAR) {
        sizes_[0] = 0;
        sizes_[1] = 0;
        sizes_[2] = 0;
    }


    Texture::~Texture() {
        if (id_ != 0) {
            glDeleteTextures(1, &id_);
            id_ = 0;
        }
    }


    Texture *Texture::create(const std::string &file_name, WrapMode wrap_mode, FilterMode filter_mode) {
        if (!file_system::is_file(file_name)) {
            LOG(ERROR) << "file does not exist: " << file_name;
            return nullptr;
        }

        int width = 0, height = 0, comp = 0;
        std::vector<unsigned char> data;
        // flip the image vertically, so the first pixel in the output array is the bottom left
        bool success = ImageIO::load(file_name, data, width, height, comp, 0, true);
        if (!success || data.empty())
            return nullptr;

        Texture *texture = create(data, width, height, comp, wrap_mode, filter_mode);
        if (texture)
            texture->name_ = file_name;

        return texture;
    }


    Texture *
    Texture::create(const std::vector<unsigned char> &rgb_data, int width, int height, int comp, WrapMode wrap_mode,
                    FilterMode filter_mode) {
        if (rgb_data.empty()) {
            LOG(ERROR) << "empty image data provided";
            return nullptr;
        } else if (rgb_data.size() != width * height * comp) {
            LOG(ERROR) << "image data does not match the given size";
            return nullptr;
        }

        GLuint tex = 0;
        glGenTextures(1, &tex); scan3d_debug_log_gl_error;
        if (tex)
            LOG(INFO) << "a texture generated from an image file, with id: " << tex;
        else {
            LOG(ERROR) << "failed to generate an OpenGL texture";
            return nullptr;
        }


        GLenum wrap, filter;
        switch (wrap_mode) {
            case CLAMP_TO_EDGE:
                wrap = GL_CLAMP_TO_EDGE;
                break;
            case REPEAT:
                wrap = GL_REPEAT;
                break;
            default: {
                LOG(ERROR) << "invalid wrap mode";
                return nullptr;
            }
        }
        switch (filter_mode) {
            case LINEAR:
                filter = GL_LINEAR;
                break;
            case NEAREST:
                filter = GL_NEAREST;
                break;
            default: {
                LOG(ERROR) << "invalid filter mode";
                return nullptr;
            }
        }

        GLenum internal_format, format;
        switch (comp) {
            case 4:
                internal_format = GL_RGBA8;
                format = GL_RGBA;
                break;
            case 3:
                internal_format = GL_RGB8;
                format = GL_RGB;
                break;
            case 2:
                internal_format = GL_RG8;
                format = GL_RG;
                break;
            case 1:
                internal_format = GL_R8;
                format = GL_RED;
                break;
            default: {
                LOG(ERROR) << "invalid format";
                return nullptr;
            }
        }

        glBindTexture(GL_TEXTURE_2D, tex);
        scan3d_debug_log_gl_error;

        // To be robust to handle
        // - R, RG or RGB textures which are not 4-bytes floats, or the width is not divisible by 4.
        // - image pixels are tightly packed.
        // Liangliang: I need to restore the modified OpenGL state
        int align;
        glGetIntegerv(GL_UNPACK_ALIGNMENT, &align);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
        scan3d_debug_log_gl_error;
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
        scan3d_debug_log_gl_error;
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
        scan3d_debug_log_gl_error;
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
        scan3d_debug_log_gl_error;

        glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height, 0, format, GL_UNSIGNED_BYTE, rgb_data.data());
        scan3d_debug_log_gl_error;
        glBindTexture(GL_TEXTURE_2D, 0);
        scan3d_debug_log_gl_error;

        glPixelStorei(GL_UNPACK_ALIGNMENT, align);

        Texture *texture = new Texture;
        texture->wrap_mode_ = wrap_mode;
        texture->filter_mode_ = filter_mode;

        texture->sizes_[0] = width;
        texture->sizes_[1] = height;
        texture->sizes_[2] = comp;
        texture->id_ = tex;

        return texture;
    }


    void Texture::bind(int unit) {
        glActiveTexture(GL_TEXTURE0 + unit);
        glBindTexture(GL_TEXTURE_2D, id_);
    }


    void Texture::release() {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, 0);
    }


    void discretize_image(std::vector<unsigned char> &data, int width, int height, int channels, int num_stripes) {
        if (num_stripes >= width)
            return;

        const int stride = width / num_stripes;
        for (int i = 0; i < num_stripes; ++i) {
            for (int j = 0; j < height; ++j) {
                for (int m = 0; m < stride; ++m)
                    for (int k = 0; k < channels; ++k)
                        data[j * (width * channels) + i * (stride * channels) + m * channels + k] =
                                data[j * (width * channels) + i * stride * channels + k];
            }
        }
    }


} // namespace scan3d
