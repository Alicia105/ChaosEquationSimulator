#ifndef PARTICLE_HPP
#define PARTICLE_HPP

struct Particle{
    float position;
    float speed;
    glm::vec3 color; 
    Particle() : position(0), speed(0){}
    Particle(float alpha, float s) : position(alpha), speed(s) {}
};

std::vector<Particle> createParticles(int numParticles,int numPoints);
glm::vec3 getInterpolatedPosition(vector<glm::vec3>& path, float t);

#endif