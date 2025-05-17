cmake_minimum_required(VERSION 3.10)
project(ChaosEquationSimulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add OpenGL
find_package(OpenGL REQUIRED)

# Add external dependencies
add_subdirectory(external/glfw)

# Source files
add_executable(ChaosEquationSimulator 
    src/main.cpp 
    src/equations.cpp
    src/shader_loader.cpp 
    external/glad/src/glad.c
)

# Include directories
target_include_directories(ChaosEquationSimulator PRIVATE
    ${OPENGL_INCLUDE_DIRS}
    external/glad/include
    external/glfw/include
    external/glm             
)

# Link libraries
target_link_libraries(ChaosEquationSimulator
    glfw
    OpenGL::GL
)
