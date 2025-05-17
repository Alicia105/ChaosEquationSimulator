#ifndef EQUATIONS_HPP
#define EQUATIONS_HPP

#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include <fstream>
#include <vector>
#include <ctime>
#include <string>
#include <glm/glm.hpp>

struct Point {
    float x, y, z;
    Point() : x(0), y(0), z(0) {}
    Point(float x, float y, float z) : x(x), y(y), z(z) {}
};

//good
std::vector<float> lorenz(Point point,float dt, float sigma, float rho, float beta);
std::vector<std::vector<float>> lorenz_trajectory(Point initialPoint,int numPoints,float maxTime, float sigma, float rho, float beta);
glm::vec3 lorenz(const glm::vec3& point, float dt, float sigma, float rho, float beta);
std::vector<glm::vec3> lorenz_trajectory(glm::vec3 initialPoint,int numPoints,float maxTime, float sigma, float rho, float beta);

//good
std::vector<float> rossler(Point point,float dt, float a, float b, float c);
std::vector<std::vector<float>> rossler_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b, float c);

//to test
glm::vec3 rossler(const glm::vec3& point,float dt, float a, float b, float c);
std::vector<glm::vec3> rossler_trajectory(glm::vec3 initialPoint,int numPoints,float maxTime, float a, float b, float c);

//good
std::vector<float> dequan_li(Point point,float dt,float a, float b, float c, float d, float e, float f);
std::vector<std::vector<float>> dequan_li_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b, float c, float d, float e, float f);

//to test
glm::vec3 dequan_li(const glm::vec3& point,float dt,float a, float b, float c, float d, float e, float f);
std::vector<glm::vec3> dequan_li_trajectory(glm::vec3 initialPoint,int numPoints,float maxTime, float a, float b, float c, float d, float e, float f);

//good
std::vector<float> aizawa(Point point,float dt,float a, float b, float c, float d, float e, float f);
std::vector<std::vector<float>> aizawa_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b, float c, float d, float e, float f);

//to test
glm::vec3 aizawa(const glm::vec3& point,float dt,float a, float b, float c, float d, float e, float f);
std::vector<glm::vec3> aizawa_trajectory(glm::vec3 initialPoint,int numPoints,float maxTime, float a, float b, float c, float d, float e, float f);

//good
std::vector<float> chen_lee(Point point,float dt,float a, float b, float d);
std::vector<std::vector<float>> chen_lee_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b,float d);

//to test
glm::vec3 chen_lee(const glm::vec3&  point,float dt,float a, float b, float d);
std::vector<glm::vec3> chen_lee_trajectory(glm::vec3 initialPoint,int numPoints,float maxTime, float a, float b,float d);

//good
std::vector<float> arneodo(Point point,float dt,float a, float b, float c);
std::vector<std::vector<float>>arneodo_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b,float c);

//to test
glm::vec3 arneodo(const glm::vec3& point,float dt,float a, float b, float c);
std::vector<glm::vec3> arneodo_trajectory(glm::vec3 initialPoint,int numPoints,float maxTime, float a, float b,float c);

//good
std::vector<float> sprott_b(Point point,float dt,float a, float b, float c);
std::vector<std::vector<float>> sprott_b_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b,float c);

//to test
glm::vec3 sprott_b(const glm::vec3& point,float dt,float a, float b, float c);
std::vector<glm::vec3> sprott_b_trajectory(glm::vec3 initialPoint,int numPoints,float maxTime, float a, float b,float c);

//good
std::vector<float> sprott_linz_f(Point point,float dt,float a);
std::vector<std::vector<float>> sprott_linz_f_trajectory(Point initialPoint,int numPoints,float maxTime, float a);

//to test
glm::vec3 sprott_linz_f(const glm::vec3& point,float dt,float a);
std::vector<glm::vec3> sprott_linz_f_trajectory(glm::vec3 initialPoint,int numPoints,float maxTime, float a);

//good
std::vector<float> dadras(Point point,float dt,float p, float o,float r, float c, float e);
std::vector<std::vector<float>> dadras_trajectory(Point initialPoint,int numPoints,float maxTime, float p, float o,float r, float c, float e);

//to test
glm::vec3 dadras(const glm::vec3& point,float dt,float p, float o,float r, float c, float e);
std::vector<glm::vec3> dadras_trajectory(glm::vec3 initialPoint,int numPoints,float maxTime, float p, float o,float r, float c, float e);

//good
std::vector<float> halvorsen(Point point,float dt,float a);
std::vector<std::vector<float>> halvorsen_trajectory(Point initialPoint,int numPoints,float maxTime, float a);

//to test
glm::vec3 halvorsen(const glm::vec3& point,float dt,float a);
std::vector<glm::vec3> halvorsen_trajectory(glm::vec3 initialPoint,int numPoints,float maxTime, float a);

#endif