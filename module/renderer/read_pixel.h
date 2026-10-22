#ifndef SCAN3D_RENDERER_OPENGL_READ_PIXEL_H
#define SCAN3D_RENDERER_OPENGL_READ_PIXEL_H

#include <string>
#include <vector>


namespace scan3d {

	namespace opengl {

		// NOTE: For the multisample-version functions, a normal fbo will be created for bliting color/depth from the 
		//       multisample framebuffer object. So for high-frequent queries, you'd better create a normal fbo for 
		//		 bliting operations. This way, you can avoid frequently allocating and deallocating GPU memories. But, 
		//		 do remember to do the cleaning when all queries are done.

		// read color value at pixel (x, y) from current fbo.
		// (x, y): the pixel coordinates, which are in the OpenGL coordinate system. 
		void read_color(unsigned char rgba[4], int x, int y);
		void read_color_ms(int index, unsigned char rgba[4], int x, int y);	// multisample framebuffer object

		// read depth value at pixel (x, y) from current fbo.
		// (x, y): the pixel coordinates, which are in the OpenGL coordinate system. 
		void read_depth(float& depth, int x, int y);
		void read_depth_ms(float& depth, int x, int y);		// multisample framebuffer object

		// read the color data of the framebuffer into a specified buffer.
		// format: format of the pixel data. The following formats are accepted: GL_RGB, GL_BGR, GL_RGBA, and GL_BGRA.
		void read_color(int index, std::vector<unsigned char>& buffer, unsigned int format, bool flip_vertically = true);
		void read_color_ms(int index, std::vector<unsigned char>& buffer, unsigned int format, bool flip_vertically = true); // multisample framebuffer object

		// read the depth data of the framebuffer into a specified buffer.
		void read_depth(std::vector<float>& buffer, bool flip_vertically = true);
		void read_depth_ms(std::vector<float>& buffer, bool flip_vertically = true); // multisample framebuffer object

		//////////////////////////////////////////////////////////////////////////

        // snapshot the color render buffer attached to color attachment index into an image file.
        // This is very useful for debugging.
        // Only png, jpg, bmp, tga, ppm are supported. File format is determined by the given extension.
        void snapshot_color(const std::string& file_name);
        void snapshot_color_ms(int index, const std::string& file_name);	// multisample framebuffer object

        // snapshot the depth render buffer into an image file. This is very useful for debugging.
        // Only png, jpg, bmp, tga, ppm are supported. File format is determined by the given extension.
        void snapshot_depth(const std::string& file_name);
        void snapshot_depth_ms(const std::string& file_name);	// multisample framebuffer object

	}

}

#endif // SCAN3D_RENDERER_OPENGL_READ_PIXEL_H


