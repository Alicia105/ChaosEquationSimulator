#ifndef SHADER_LOADER_HPP
#define SHADER_LOADER_HPP

#include <string>
#include <GLFW/glfw3.h>
#include <fstream>
#include <sstream>
#include <iostream>

GLuint LoadShaders(const char* vertex_path, const char* fragment_path);

#endif
