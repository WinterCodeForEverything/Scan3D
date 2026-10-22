#include <module/renderer/vertex_array_object.h>

#include <cassert>

#include <module/renderer/opengl_error.h>
#include <module/renderer/opengl_info.h>
#include <module/util/logging.h>


namespace scan3d {


    bool VertexArrayObject::is_supported() {
        return OpenglInfo::is_supported("GL_VERSION_2_1") ||
            OpenglInfo::is_supported("GL_ARB_vertex_array_object") ||
            OpenglInfo::has_entension("GL_EXT_vertex_array_object");
    }


	VertexArrayObject::VertexArrayObject()
		: id_(0)
	{
        if (!is_supported()) {
            LOG(ERROR) << "vertex array object not supported on this platform";
            return;
        }

        // Liangliang: it is a bad idea to initialize OpenGL stuff in the constructors
        //			   because the OpenGL context may not exist. In Scan3D, I alway
        //             follow the "create when needed" rule.
        // glGenVertexArrays(1, &id_);	scan3d_debug_log_gl_error;
        // if (id_ == 0) {
        //     LOG(ERROR) << "generating VertexArrayObject failed";
		//}
	}


	VertexArrayObject::~VertexArrayObject() {
		if (id_ != 0) {
			glDeleteVertexArrays(1, &id_);	scan3d_debug_log_gl_error;
			id_ = 0;
		}
	}


	void VertexArrayObject::bind() {
		if (id_ == 0) {
			glGenVertexArrays(1, &id_);	scan3d_debug_log_gl_error;
			if (id_ == 0) {
				LOG(ERROR) << "generating VertexArrayObject failed";
				return;
			}
		}

		glBindVertexArray(id_);		scan3d_debug_log_gl_error;
	}


    void VertexArrayObject::release() {
		assert(id_ != 0);
		glBindVertexArray(0);		scan3d_debug_log_gl_error;
	}


    void VertexArrayObject::release_buffer(GLuint& buffer) {
        if (buffer != 0) {
			glBindVertexArray(0);			scan3d_debug_log_gl_error;
            glDeleteBuffers(1, &buffer);	scan3d_debug_log_gl_error;
            buffer = 0;
		}
	}


    bool VertexArrayObject::create_array_buffer(GLuint& buffer, GLuint index, const void* data, std::size_t size, std::size_t dim, bool dynamic) {
        release_buffer(buffer);
		bind();
        glGenBuffers(1, &buffer);                       scan3d_debug_log_gl_error;
        glBindBuffer(GL_ARRAY_BUFFER, buffer);			scan3d_debug_log_gl_error;
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(size), data, dynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);		scan3d_debug_log_gl_error;
        glEnableVertexAttribArray(index);               scan3d_debug_log_gl_error;
        glVertexAttribPointer(index, int(dim), GL_FLOAT, GL_FALSE, 0, nullptr);		scan3d_debug_log_gl_error;
        if (glGetError() != GL_NO_ERROR) {
            glBindBuffer(GL_ARRAY_BUFFER, 0);           scan3d_debug_log_gl_error;
            glDeleteBuffers(1, &buffer);                scan3d_debug_log_gl_error;
            buffer = 0;
		}
        glBindBuffer(GL_ARRAY_BUFFER, 0);               scan3d_debug_log_gl_error;
        release();
        return (glGetError() == GL_NO_ERROR);
	}


    bool VertexArrayObject::create_element_buffer(GLuint &buffer, const void *data, std::size_t size, bool dynamic) {
        release_buffer(buffer);
		bind();
        glGenBuffers(1, &buffer);                                           scan3d_debug_log_gl_error;
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer);                      scan3d_debug_log_gl_error;
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(size), data, dynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);		scan3d_debug_log_gl_error;
        if (glGetError() != GL_NO_ERROR) {
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);                       scan3d_debug_log_gl_error;
            glDeleteBuffers(1, &buffer);                                    scan3d_debug_log_gl_error;
            buffer = 0;
		}
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);	scan3d_debug_log_gl_error;
        release();
        return (glGetError() == GL_NO_ERROR);
	}


    bool VertexArrayObject::create_storage_buffer(GLuint& buffer, GLuint index, const void* data, std::size_t size) {
        if (!OpenglInfo::is_supported("GL_ARB_shader_storage_buffer_object")) {
            LOG(ERROR) << "shader storage buffer object not supported on this platform";
            return false;
        }
        release_buffer(buffer);
		bind();
        glGenBuffers(1, &buffer);                                                   scan3d_debug_log_gl_error;
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffer);                             scan3d_debug_log_gl_error;
        glBufferData(GL_SHADER_STORAGE_BUFFER, static_cast<GLsizeiptr>(size), data, GL_DYNAMIC_DRAW);		scan3d_debug_log_gl_error;
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, index, buffer);                   scan3d_debug_log_gl_error;
        if (glGetError() != GL_NO_ERROR) {
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);                              scan3d_debug_log_gl_error;
            glDeleteBuffers(1, &buffer);                                            scan3d_debug_log_gl_error;
            buffer = 0;
		}
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);                                  scan3d_debug_log_gl_error;
        release();                                                                  scan3d_debug_log_gl_error;
        return (glGetError() == GL_NO_ERROR);
	}


    bool VertexArrayObject::update_storage_buffer(GLuint& buffer, GLintptr offset, GLsizeiptr size, const void* data) {
		bind();
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffer);                     scan3d_debug_log_gl_error;
		glBufferSubData(GL_SHADER_STORAGE_BUFFER, offset, size, data);		scan3d_debug_log_gl_error;
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);                          scan3d_debug_log_gl_error;
        release();                                                          scan3d_debug_log_gl_error;
        return (glGetError() == GL_NO_ERROR);
	}


    void* VertexArrayObject::map_buffer(GLenum target, GLuint buffer, GLenum access) {
		// Liangliang: should work, but haven't tested yet.
        glBindBuffer(target, buffer);                   scan3d_debug_log_gl_error;
		// Liangliang: Finally, I found this is the place to call glMemoryBarrier()
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);	scan3d_debug_log_gl_error;
        void* ptr = glMapBuffer(buffer, access);        scan3d_debug_log_gl_error;
        glBindBuffer(target, 0);                        scan3d_debug_log_gl_error;
		return ptr;
	}


    void VertexArrayObject::unmap_buffer(GLenum target, GLuint buffer) {
		// Liangliang: should work, but haven't tested yet.
        glBindBuffer(target, buffer);	scan3d_debug_log_gl_error;
        glUnmapBuffer(buffer);          scan3d_debug_log_gl_error;
        glBindBuffer(target, 0);        scan3d_debug_log_gl_error;
	}


    void VertexArrayObject::get_buffer_data(GLenum target, GLuint buffer, GLintptr offset, GLsizeiptr size, void* data) {
        glBindBuffer(target, buffer);                   scan3d_debug_log_gl_error;
		// Liangliang: Finally, I found this is the place to call glMemoryBarrier()
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);	scan3d_debug_log_gl_error;
		glGetBufferSubData(target, offset, size, data);	scan3d_debug_log_gl_error;
        glBindBuffer(target, 0);                        scan3d_debug_log_gl_error;
	}
}
