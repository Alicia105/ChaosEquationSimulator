cmake_minimum_required(VERSION 3.10)
project(ChaosEquationSimulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add GLM
find_package(glm REQUIRED)

# Add OpenGL
find_package(OpenGL REQUIRED)

# Add GLFW
find_package(glfw3 3.3 REQUIRED)

# Add GLAD (you need to include it in the project, I’ll explain below)
add_subdirectory(external/glad)

# Source files
add_executable(ChaosEquationSimulator 
    src/main.cpp 
    src/equations.cpp 
    external/glad/src/glad.c
)

# Include directories
target_include_directories(ChaosEquationSimulator PRIVATE
    ${OPENGL_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    external/glad/include
    ${GLM_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(ChaosEquationSimulator
    glfw
    glad
    OpenGL::GL
)
