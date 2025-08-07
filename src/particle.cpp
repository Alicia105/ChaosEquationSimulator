#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <string>
#include <bits/stdc++.h>
#include "../include/particle.hpp"
#include <glm/glm.hpp>

using namespace std;

vector<Particle> createParticles(int numParticles,int numPoints){
    vector<Particle> particles;
    for(int i=0; i<numParticles; i++){
        Particle p;
        p.position = static_cast<float>(rand() % numPoints);  // random starting index
        p.speed = 0.05f + static_cast<float>(rand() % 50) / 1000.0f; // random speed ~0.05 - 0.1
        glm::vec3 color = glm::vec3(
            static_cast<float>(rand()) / RAND_MAX,
            static_cast<float>(rand()) / RAND_MAX,
            static_cast<float>(rand()) / RAND_MAX
        );      
        p.color=color;
        particles.push_back(p);
    }
    return particles;
}

glm::vec3 getInterpolatedPosition(vector<glm::vec3>& path, float t) {
    int idx = static_cast<int>(t);   // base index
    float alpha = t - idx;           // fractional part

    if (idx >= path.size() - 1) return path.back();  // end case

    return glm::mix(path[idx], path[idx + 1], alpha);
}
