#ifndef SCAN3D_RENDERER_SHADER_MANAGER_H
#define SCAN3D_RENDERER_SHADER_MANAGER_H

#include <string>
#include <unordered_map>

#include <module/renderer/shader_program.h>


// NOTE: make sure to call terminate() to destroy existing programs before the OpenGL context is deleted.


namespace scan3d {

    class ShaderProgram;

    class ShaderManager
    {
    public:
        // return the shader program if it exists and is working, otherwise return 0.
        // shader_name: the base name of the program's source file.
        static ShaderProgram* get_program(const std::string& shader_name);

        // create a shader program from shader source files specified by the shader file's base name.
        static ShaderProgram* create_program_from_files(
            const std::string& file_base_name,
            const std::vector<ShaderProgram::Attribute>& attributes = std::vector<ShaderProgram::Attribute>(),
            const std::vector<std::string>& outputs = std::vector<std::string>(),
            bool geom_shader = false
        );

        // create a shader program from shader source files specified by individual file names.
        // This function allows user providing extra code.
		static ShaderProgram* create_program_from_files(
			const std::string& vert_file_name,
			const std::string& frag_file_name,
			const std::string& geom_file_name = "",
			const std::string& extra_vert_code = "",
			const std::string& extra_frag_code = "",
			const std::string& extra_geom_code = "",
			const std::vector<ShaderProgram::Attribute>& attributes = std::vector<ShaderProgram::Attribute>(),
			const std::vector<std::string>& outputs = std::vector<std::string>()
		);

        static std::vector<ShaderProgram*> all_programs();

        // destroy all shader programs.
        static void terminate();

		static void reload();

    private:
        // maps of std::string can be super slow when calling find with a string literal or const char*
        // as find forces construction/copy/destruction of a std::sting copy of the const char*.
        static std::unordered_map<std::string, ShaderProgram*>	programs_;
        static std::unordered_map<std::string, bool>			attempt_load_program_; // avoid multiple attempt
    };

}


#endif  // SCAN3D_RENDERER_SHADER_MANAGER_H
