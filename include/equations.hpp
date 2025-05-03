#ifndef EQUATIONS_H
#define EQUATIONS_H

#include <iostream>
#include <vector>
#include <bits/stdc++.h>

struct Point;

//good
std::vector<float> lorenz(Point point,float dt, float sigma, float rho, float beta);
std::vector<std::vector<float>> lorenz_trajectory(Point initialPoint,int numPoints,float maxTime, float sigma, float rho, float beta);

//good
std::vector<float> rossler(Point point,float dt, float a, float b, float c);
std::vector<std::vector<float>> rossler_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b, float c);

//good
std::vector<float> dequan_li(Point point,float dt,float a, float b, float c, float d, float e, float f);
std::vector<std::vector<float>> dequan_li_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b, float c, float d, float e, float f);

//good
std::vector<float> aizawa(Point point,float dt,float a, float b, float c, float d, float e, float f);
std::vector<std::vector<float>> aizawa_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b, float c, float d, float e, float f);

//good
std::vector<float> chen_lee(Point point,float dt,float a, float b, float d);
std::vector<std::vector<float>> chen_lee_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b,float d);

//good
std::vector<float> arneodo(Point point,float dt,float a, float b, float c);
std::vector<std::vector<float>>arneodo_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b,float c);

//good
std::vector<float> sprott_b(Point point,float dt,float a, float b, float c);
std::vector<std::vector<float>> sprott_b_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b,float c);

//good
std::vector<float> sprott_linz_f(Point point,float dt,float a);
std::vector<std::vector<float>> sprott_linz_f_trajectory(Point initialPoint,int numPoints,float maxTime, float a);

//good
std::vector<float> dadras(Point point,float dt,float p, float o,float r, float c, float e);
std::vector<std::vector<float>> dadras_trajectory(Point initialPoint,int numPoints,float maxTime, float p, float o,float r, float c, float e);

//good
std::vector<float> halvorsen(Point point,float dt,float a);
std::vector<std::vector<float>> halvorsen_trajectory(Point initialPoint,int numPoints,float maxTime, float a);


#endif