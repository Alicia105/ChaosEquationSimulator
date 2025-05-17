#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "../include/shader_loader.hpp"

using namespace std;

GLuint LoadShaders(const char* vertex_path, const char* fragment_path) {
    string vertex_code;
    string fragment_code;
    ifstream v_shader_file;
    ifstream f_shader_file;

    // Ensure ifstream objects can throw exceptions:
    v_shader_file.exceptions(ifstream::failbit | ifstream::badbit);
    f_shader_file.exceptions(ifstream::failbit | ifstream::badbit);

    try {
        v_shader_file.open(vertex_path);
        f_shader_file.open(fragment_path);
        stringstream v_shader_stream, f_shader_stream;

        v_shader_stream << v_shader_file.rdbuf();
        f_shader_stream << f_shader_file.rdbuf();

        v_shader_file.close();
        f_shader_file.close();

        vertex_code = v_shader_stream.str();
        fragment_code = f_shader_stream.str();
    } catch (ifstream::failure& e) {
        cerr << "ERROR: Shader file not successfully read" << endl;
        return 0;
    }

    const char* v_shader_code = vertex_code.c_str();
    const char* f_shader_code = fragment_code.c_str();

    GLuint vertex, fragment;
    GLint success;
    GLchar info_log[512];

    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &v_shader_code, nullptr);
    glCompileShader(vertex);
    glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertex, 512, nullptr, info_log);
        cerr << "ERROR: Vertex shader compilation failed\n" << info_log << endl;
    }

    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &f_shader_code, nullptr);
    glCompileShader(fragment);
    glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragment, 512, nullptr, info_log);
        cerr << "ERROR: Fragment shader compilation failed\n" << info_log << endl;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(program, 512, nullptr, info_log);
        cerr << "ERROR: Shader program linking failed\n" << info_log << endl;
    }

    glDeleteShader(vertex);
    glDeleteShader(fragment);

    return program;
}

