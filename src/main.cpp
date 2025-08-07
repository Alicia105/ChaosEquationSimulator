#include <vector>
#include <iostream>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "../include/equations.hpp"
#include "../include/particle.hpp"
#include "../include/shader_loader.hpp"

using namespace std;

//Constants for camera control
float cameraDistance = 50.0f;
float yaw = -90.0f;  // left/right
float pitch = 0.0f;  // up/down

float lastX = 400;
float lastY = 300;

bool leftMouse = true;
bool mouseHeld = false;
bool rightMouse = false;

float radius = 50.0f;
double lastPanX = 0.0;
double lastPanY = 0.0;
glm::vec3 cameraTarget = glm::vec3(0.0f);

//Attractors 
enum AttractorType {
    LORENZ,
    ROSSLER,
    DEQUAN_LI,
    AIZAWA,
    CHEN_LEE,
    ARNEODO,
    SPROTT_B,
    SPROTT_LINZ_F,
    DADRAS,
    HALVORSEN,
    THOMAS,
    LORENZ83,
    RABINOVICH_FABRIKANT,
    FOUR_WING,
    SPROTT
};
AttractorType currentAttractor = LORENZ;

float lorenzSigma = 10;
float lorenzRho = 28;
float lorenzBeta = 8/3; 

float rosslerA = 0.1;
float rosslerB = 0.2;
float rosslerC = 5.7;

float dequan_liA = 41;
float dequan_liB = 11;
float dequan_liC = 0.16;
float dequan_liD = 0.65;
float dequan_liE = 55;
float dequan_liF = 20;

float aizawaA = 0.95;
float aizawaB = 0.7;
float aizawaC = 0.6;
float aizawaD = 3.5;
float aizawaE = 0.25;
float aizawaF = 0.1;

float chen_leeA = 5;
float chen_leeB = -10;
float chen_leeD = -0.38;

float arneodoA = -5.5;
float arneodoB = 3.5;
float arneodoC = -1;

float sprott_bA = 0.4;
float sprott_bB = 1.2;
float sprott_bC = 1;

float sprott_linz_fA = 0.5;

float dadrasP = 3;
float dadrasO = 2.7;
float dadrasR = 1.7;
float dadrasC = 2;
float dadrasE = 9;

float halvorsenA = 1.4;

float thomasB = 0.208186;

float lorenz83A = 0.95;
float lorenz83B = 7.91;
float lorenz83F = 4.83;
float lorenz83G = 4.66;

float rabinovichAlpha=0.14;
float rabinovichGamma=0.10;

float fourWingA=0.2;
float fourWingB=0.01;
float fourWingC=-0.4;

float sprottA=2.07;
float sprottB=1.79;

//initial trajectory characteristics
int numPoints=1000;
float maxTime=5;
float dt=maxTime/numPoints;
float x=5.0f;
float y=5.0f;
float z=5.0f;
int numParticles=10;
bool addingParticles=false;

//Generate trajectory

vector<glm::vec3> generate_trajectory(AttractorType type, const glm::vec3& start,int numPoints,float maxTime) {
    if (type == LORENZ)
        return lorenz_trajectory(start, numPoints,maxTime, lorenzSigma, lorenzRho, lorenzBeta);
    else if (type == ROSSLER)
        return rossler_trajectory(start, numPoints,maxTime, rosslerA, rosslerB, rosslerC);
    else if (type ==DEQUAN_LI)
        return dequan_li_trajectory(start,numPoints,maxTime, dequan_liA, dequan_liB, dequan_liC, dequan_liD, dequan_liE,dequan_liF);
    else if (type == AIZAWA)
        return aizawa_trajectory(start,numPoints,maxTime, aizawaA, aizawaB, aizawaC, aizawaD, aizawaE, aizawaF);
    else if (type == CHEN_LEE)
        return chen_lee_trajectory(start,numPoints,maxTime, chen_leeA,chen_leeB,chen_leeD);
    else if (type == ARNEODO)
        return arneodo_trajectory(start,numPoints,maxTime,arneodoA,arneodoB,arneodoC);
    else if (type == SPROTT_B)
        return sprott_b_trajectory(start,numPoints,maxTime,sprott_bA,sprott_bB,sprott_bC);
    else if (type == SPROTT_LINZ_F)
        return sprott_linz_f_trajectory(start,numPoints,maxTime,sprott_linz_fA);
    else if (type == DADRAS)
        return dadras_trajectory(start,numPoints,maxTime, dadrasP, dadrasO,dadrasR,dadrasC,dadrasE);
    else if (type == HALVORSEN)
        return halvorsen_trajectory(start,numPoints,maxTime, halvorsenA);
    else if (type == THOMAS)
        return thomas_trajectory(start,numPoints,maxTime, thomasB);
    else if (type == LORENZ83)
        return lorenz83_trajectory(start,numPoints,maxTime,lorenz83A,lorenz83B,lorenz83F,lorenz83G);
    else if (type == RABINOVICH_FABRIKANT)
        return rabinovich_fabrikant_trajectory(start,numPoints,maxTime,rabinovichAlpha,rabinovichGamma);
    else if (type == FOUR_WING)
        return four_wing_trajectory(start,numPoints,maxTime,fourWingA,fourWingB,fourWingC);
    else if (type == SPROTT)
        return sprott_trajectory(start,numPoints,maxTime,sprottA,sprottB);
    return {};
}

vector<vector<float>> generate_trajectory_csv(AttractorType type, Point start,int numPoints,float maxTime) {
    if (type == LORENZ)
        return lorenz_trajectory(start, numPoints,maxTime, lorenzSigma, lorenzRho, lorenzBeta);
    else if (type == ROSSLER)
        return rossler_trajectory(start, numPoints,maxTime, rosslerA, rosslerB, rosslerC);
    else if (type ==DEQUAN_LI)
        return dequan_li_trajectory(start,numPoints,maxTime, dequan_liA, dequan_liB, dequan_liC, dequan_liD, dequan_liE,dequan_liF);
    else if (type == AIZAWA)
        return aizawa_trajectory(start,numPoints,maxTime, aizawaA, aizawaB, aizawaC, aizawaD, aizawaE, aizawaF);
    else if (type == CHEN_LEE)
        return chen_lee_trajectory(start,numPoints,maxTime, chen_leeA,chen_leeB,chen_leeD);
    else if (type == ARNEODO)
        return arneodo_trajectory(start,numPoints,maxTime,arneodoA,arneodoB,arneodoC);
    else if (type == SPROTT_B)
        return sprott_b_trajectory(start,numPoints,maxTime,sprott_bA,sprott_bB,sprott_bC);
    else if (type == SPROTT_LINZ_F)
        return sprott_linz_f_trajectory(start,numPoints,maxTime,sprott_linz_fA);
    else if (type == DADRAS)
        return dadras_trajectory(start,numPoints,maxTime, dadrasP, dadrasO,dadrasR,dadrasC,dadrasE);
    else if (type == HALVORSEN)
        return halvorsen_trajectory(start,numPoints,maxTime, halvorsenA);
    else if (type == THOMAS)
        return thomas_trajectory(start,numPoints,maxTime, thomasB);
    else if (type == LORENZ83)
        return lorenz83_trajectory(start,numPoints,maxTime,lorenz83A,lorenz83B,lorenz83F,lorenz83G);
    else if (type == RABINOVICH_FABRIKANT)
        return rabinovich_fabrikant_trajectory(start,numPoints,maxTime,rabinovichAlpha,rabinovichGamma);
    else if (type == FOUR_WING)
        return four_wing_trajectory(start,numPoints,maxTime,fourWingA,fourWingB,fourWingC);
    else if (type == SPROTT)
        return sprott_trajectory(start,numPoints,maxTime,sprottA,sprottB);
    return {};
}

/*vector<glm::vec3> drawParticles(AttractorType type, const glm::vec3& start,int numPoints,float maxTime){

}*/

float computeSpeed(glm::vec3& point, AttractorType currentAttractor){
    glm::vec3 vit;

    if (currentAttractor == LORENZ) {
        vit=speed_lorenz(point,lorenzSigma,lorenzRho,lorenzBeta);
    } 
    else if (currentAttractor == ROSSLER) {
        vit=speed_rossler(point,rosslerA,rosslerB,rosslerC);
    } 
    else if (currentAttractor == AIZAWA) {
        vit=speed_aizawa(point,aizawaA,aizawaB,aizawaC,aizawaD,aizawaE,aizawaF);
    }
    else if (currentAttractor == DEQUAN_LI){
        vit=speed_dequan_li(point,dequan_liA,dequan_liB,dequan_liC,dequan_liD,dequan_liE,dequan_liF);
    }
    else if (currentAttractor == CHEN_LEE){
        vit=speed_chen_lee(point,chen_leeA,chen_leeB,chen_leeD);
    }
    else if (currentAttractor == ARNEODO){
        vit=speed_arneodo(point,arneodoA,arneodoB,arneodoC);
    }
    else if (currentAttractor == SPROTT_B){
        vit=speed_sprott_b(point,sprott_bA,sprott_bB,sprott_bC);
    }
    else if (currentAttractor == SPROTT_LINZ_F){
        vit=speed_sprott_linz_f(point,sprott_linz_fA);
    }
    else if (currentAttractor == DADRAS){
        vit=speed_dadras(point,dadrasP,dadrasO,dadrasR,dadrasC,dadrasE);
    }
    else if (currentAttractor == HALVORSEN){
        vit=speed_halvorsen(point,halvorsenA);
    }
    else if (currentAttractor == THOMAS){
        vit=speed_thomas(point,thomasB);
    }
    else if (currentAttractor == LORENZ83){
        vit=speed_lorenz83(point,lorenz83A,lorenz83B,lorenz83F,lorenz83G);
    }
    else if (currentAttractor == RABINOVICH_FABRIKANT){
        vit=speed_rabinovich_fabrikant(point,rabinovichAlpha,rabinovichGamma);                
    }
    else if (currentAttractor == FOUR_WING){
        vit=speed_four_wing(point,fourWingA,fourWingB,fourWingC);
    }
    else if (currentAttractor == SPROTT){
        vit=speed_sprott(point,sprottA,sprottB);
    }

    return speed_norm(vit);

}

glm::vec3 getPositionAt(glm::vec3 initialPoint, float timeDuration, AttractorType currentAttractor){
    glm::vec3 point=initialPoint;
    float dt=0.01f;
    int steps = static_cast<int>(timeDuration / dt);
    int c=0;

    if (currentAttractor == LORENZ) {
        for (int i = 0; i < steps; i++) {
            point+=lorenz(point,dt, lorenzSigma,lorenzRho,lorenzBeta);
            c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=lorenz(point,dt, lorenzSigma,lorenzRho,lorenzBeta);
        }
    } 
    else if (currentAttractor == ROSSLER) {
        for (int i = 0; i < steps; i++) {
            point+=rossler(point,dt, rosslerA,rosslerB,rosslerC);
            c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=rossler(point,dt, rosslerA,rosslerB,rosslerC);
        }      
    } 
    else if (currentAttractor == AIZAWA) {
        for (int i = 0; i < steps; i++) {
            point+=aizawa(point,dt,aizawaA,aizawaB,aizawaC,aizawaD,aizawaE,aizawaF);
            c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=aizawa(point,dt,aizawaA,aizawaB,aizawaC,aizawaD,aizawaE,aizawaF);
        }        
    }
    else if (currentAttractor == DEQUAN_LI){
        for (int i = 0; i < steps; i++) {
            point+=dequan_li(point,dt,dequan_liA,dequan_liB,dequan_liC,dequan_liD,dequan_liE,dequan_liF);
            c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=dequan_li(point,dt,dequan_liA,dequan_liB,dequan_liC,dequan_liD,dequan_liE,dequan_liF);
        }        
    }
    else if (currentAttractor == CHEN_LEE){
        for (int i = 0; i < steps; i++) {
            point+=chen_lee(point,dt,chen_leeA,chen_leeB,chen_leeD);
            c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=chen_lee(point,dt,chen_leeA,chen_leeB,chen_leeD);
        }
    }
    else if (currentAttractor == ARNEODO){
        for (int i = 0; i < steps; i++) {
            point+=arneodo(point,dt,arneodoA,arneodoB,arneodoC);
            c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=arneodo(point,dt,arneodoA,arneodoB,arneodoC);
        }
    }
    else if (currentAttractor == SPROTT_B){
        for (int i = 0; i < steps; i++) {
            point+=sprott_b(point,dt,sprott_bA,sprott_bB,sprott_bC);
            c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=sprott_b(point,dt,sprott_bA,sprott_bB,sprott_bC);
        }
    }
    else if (currentAttractor == SPROTT_LINZ_F){
        for (int i = 0; i < steps; i++) {
            point+=sprott_linz_f(point,dt,sprott_linz_fA);
            c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=sprott_linz_f(point,dt,sprott_linz_fA);
        }
    }
    else if (currentAttractor == DADRAS){
        for (int i = 0; i < steps; i++) {
            point+=dadras(point,dt,dadrasP,dadrasO,dadrasR,dadrasC,dadrasE);
            c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=dadras(point,dt,dadrasP,dadrasO,dadrasR,dadrasC,dadrasE);
        }
    }
    else if (currentAttractor == HALVORSEN){
        for (int i = 0; i < steps; i++) {
            point+=halvorsen(point,dt,halvorsenA);
            c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=halvorsen(point,dt,halvorsenA);
        }       
    }
    else if (currentAttractor == THOMAS){
        for (int i = 0; i < steps; i++) {
            point+=thomas(point,dt,thomasB);
            c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=thomas(point,dt,thomasB);
        }
    }
    else if (currentAttractor == LORENZ83){
        for (int i = 0; i < steps; i++) {
            point+=lorenz83(point,dt,lorenz83A,lorenz83B,lorenz83F,lorenz83G);
            c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=lorenz83(point,dt,lorenz83A,lorenz83B,lorenz83F,lorenz83G);
        }
    }
    else if (currentAttractor == RABINOVICH_FABRIKANT){
        for (int i = 0; i < steps; i++) {
            point+=rabinovich_fabrikant(point,dt,rabinovichAlpha,rabinovichGamma);
            c=i+1;  
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=rabinovich_fabrikant(point,dt,rabinovichAlpha,rabinovichGamma);
        }        
    }
    else if (currentAttractor == FOUR_WING){
        for (int i = 0; i < steps; i++) {
           point+=four_wing(point,dt,fourWingA,fourWingB,fourWingC);
           c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=four_wing(point,dt,fourWingA,fourWingB,fourWingC);
        }
    }
    else if (currentAttractor == SPROTT){
        for (int i = 0; i < steps; i++) {
            point+=sprott(point,dt,sprottA,sprottB);
            c=i+1;
        }
        if(c*dt<=timeDuration){
            dt=timeDuration-c*dt;
            point+=sprott(point,dt,sprottA,sprottB);
        }
    }

    return point;
}

//Actions window call back

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    cameraDistance -= yoffset * 2.0f;
    if (cameraDistance < 2.0f) cameraDistance = 2.0f;
    if (cameraDistance > 200.0f) cameraDistance = 200.0f;
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    if (!mouseHeld) return;

    if (rightMouse) {
        float xoffset = lastPanX - xpos ;
        float yoffset = lastPanY - ypos;
        lastPanX = xpos;
        lastPanY = ypos;

        float panSpeed = 0.1f;

        glm::vec3 front;
        front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
        front.y = sin(glm::radians(pitch));
        front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
        glm::vec3 direction = glm::normalize(front);
        glm::vec3 right = glm::normalize(glm::cross(direction, glm::vec3(0.0f, 1.0f, 0.0f)));
        glm::vec3 up = glm::normalize(glm::cross(right, direction));

        cameraTarget -= right * xoffset * panSpeed;
        cameraTarget -= up * yoffset * panSpeed;

    } else {
        if (leftMouse) {
            lastX = xpos;
            lastY = ypos;
            leftMouse = false;
        }

        float xoffset = xpos - lastX;
        float yoffset = lastY - ypos;
        lastX = xpos;
        lastY = ypos;

        float sensitivity = 0.1f;
        xoffset *= sensitivity;
        yoffset *= sensitivity;

        yaw += xoffset;
        pitch += yoffset;

        if (pitch > 89.0f) pitch = 89.0f;
        if (pitch < -89.0f) pitch = -89.0f;
    }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            mouseHeld = true;
            leftMouse = true;
        } else if (action == GLFW_RELEASE) {
            mouseHeld = false;
        }
    }

    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            rightMouse = true;
            mouseHeld = true;
            glfwGetCursorPos(window, &lastPanX, &lastPanY);
        } else if (action == GLFW_RELEASE) {
            rightMouse = false;
            mouseHeld = false;
        }
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_R && action == GLFW_PRESS) {
        yaw = -90.0f;
        pitch = 0.0f;
        radius = 50.0f;
        cameraTarget = glm::vec3(0.0f);
    }
}

int main() {

    if (!glfwInit()) {
        cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(800, 600, "Chaos Equation Simulator", nullptr, nullptr);
    if (!window) {
        cerr << "Failed to create window\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetKeyCallback(window, key_callback);


    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        cerr << "Failed to initialize GLAD\n";
        return -1;
    }

    glEnable(GL_DEPTH_TEST);
    glLineWidth(2.0f);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    //initialize first attractor
    glm::vec3 initialPoint(x,y,z);
    vector<glm::vec3> trajectory=generate_trajectory(currentAttractor,initialPoint,numPoints,maxTime);
    cout << "Trajectory size: " << trajectory.size() << endl;

    vector<Particle> particles;
    vector<glm::vec3> particlePositions;
    vector<glm::vec3> particleColors;

    //Buffers
    GLuint VAO, VBO,particleVAO,particlePosVBO,particleColorVBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    
    glGenVertexArrays(1, &particleVAO);
    glGenBuffers(1, &particlePosVBO);
    glGenBuffers(1, &particleColorVBO);


    //Trajectory setup
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, trajectory.size() * sizeof(glm::vec3), trajectory.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    //Particle VAO setup
    glBindVertexArray(particleVAO);
    
    //Particle positions
    glBindBuffer(GL_ARRAY_BUFFER, particlePosVBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);
    
    //Particle colors
    glBindBuffer(GL_ARRAY_BUFFER, particleColorVBO);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);



    GLuint shaderProgram = LoadShaders("../shaders/vertex.glsl", "../shaders/fragment.glsl");
    glUseProgram(shaderProgram);
    cout << "Shader program ID: " << shaderProgram << endl;

    // Enable point drawing
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(5.0f); // You can animate or modify this later

    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // === UI Code ===
        ImGui::Begin("Attractor Settings");

        // Shared initial point input
        if (ImGui::CollapsingHeader("Initial Point")) {
            ImGui::InputFloat("X", &initialPoint.x, 0.1f, 1.0f, "%.2f");
            ImGui::InputFloat("Y", &initialPoint.y, 0.1f, 1.0f, "%.2f");
            ImGui::InputFloat("Z", &initialPoint.z, 0.1f, 1.0f, "%.2f");
            ImGui::InputInt("Number of points", &numPoints, 1, 1);
            ImGui::InputFloat("Time duration (in sec)", &maxTime, 0.1f, 1.0f, "%.2f");
            dt=maxTime/numPoints;
        }

        if (ImGui::CollapsingHeader("Choose Attractor", ImGuiTreeNodeFlags_DefaultOpen)) {
            const char* items[] = { "Lorenz","Rossler","Dequan_Li","Aizawa","Chen_Lee","Arneodo","Sprott_B","Sprott_Linz_F","Dadras","Halvorsen","Thomas","Lorenz83","Rabinovich_Fabrikant","Four_Wing","Sprott"};
            static int selected = currentAttractor;
            if (ImGui::Combo("Attractor", &selected, items, IM_ARRAYSIZE(items))) {
                currentAttractor = static_cast<AttractorType>(selected);
            }
        }

        // Parameters per attractor
        if (ImGui::CollapsingHeader("Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (currentAttractor == LORENZ) {
                ImGui::InputFloat("Sigma", &lorenzSigma, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("Rho", &lorenzRho, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("Beta", &lorenzBeta, 0.1f, 1.0f, "%.3f");
            } 
            else if (currentAttractor == ROSSLER) {
                ImGui::InputFloat("a", &rosslerA, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("b", &rosslerB, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("c", &rosslerC, 0.1f, 1.0f, "%.3f");
            } 
            else if (currentAttractor == DEQUAN_LI){
                ImGui::InputFloat("a",&dequan_liA, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("b", &dequan_liB, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("c", &dequan_liC, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("d", &dequan_liD, 0.1f, 1.0f, "%.3f"); 
                ImGui::InputFloat("e", &dequan_liE, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("f", &dequan_liF, 0.1f, 1.0f, "%.3f");
            }
            else if (currentAttractor == AIZAWA){
                ImGui::InputFloat("a", &aizawaA, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("b", &aizawaB, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("c", &aizawaC, 0.1f, 1.0f, "%.3f"); 
                ImGui::InputFloat("d", &aizawaD, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("e", &aizawaE, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("f", &aizawaF, 0.1f, 1.0f, "%.3f"); 
            }
            else if (currentAttractor == CHEN_LEE){
                ImGui::InputFloat("a", &chen_leeA, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("b", &chen_leeB, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("d", &chen_leeD, 0.1f, 1.0f, "%.3f");
            }
            else if (currentAttractor == ARNEODO){
                ImGui::InputFloat("a", &arneodoA, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("b", &arneodoB, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("c", &arneodoC, 0.1f, 1.0f, "%.3f");
            }
            else if (currentAttractor == SPROTT_B){
                ImGui::InputFloat("a", &sprott_bA, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("b", &sprott_bB, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("c", &sprott_bC, 0.1f, 1.0f, "%.3f");
            }
            else if (currentAttractor == SPROTT_LINZ_F){
                ImGui::InputFloat("a", &sprott_linz_fA, 0.1f, 1.0f, "%.3f");
            }
            else if (currentAttractor == DADRAS){
                ImGui::InputFloat("p", &dadrasP, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("o", &dadrasO, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("r", &dadrasR, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("c", &dadrasC, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("e", &dadrasE, 0.1f, 1.0f, "%.3f");
            }
            else if (currentAttractor == HALVORSEN){
                ImGui::InputFloat("a", &halvorsenA, 0.1f, 1.0f, "%.3f");
            }
            else if (currentAttractor == THOMAS){
                ImGui::InputFloat("b", &thomasB, 0.1f, 1.0f, "%.3f");
            }
            else if (currentAttractor == LORENZ83){
                ImGui::InputFloat("a", &lorenz83A, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("b", &lorenz83B, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("f", &lorenz83F, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("g", &lorenz83G, 0.1f, 1.0f, "%.3f");
            }
            else if (currentAttractor == RABINOVICH_FABRIKANT){
                ImGui::InputFloat("alpha", &rabinovichAlpha, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("gamma", &rabinovichGamma, 0.1f, 1.0f, "%.3f");                
            }
            else if (currentAttractor == FOUR_WING){
                ImGui::InputFloat("a", &fourWingA, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("b", &fourWingB, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("c", &fourWingC, 0.1f, 1.0f, "%.3f");
            }
            else if (currentAttractor == SPROTT){
                ImGui::InputFloat("a", &sprottA, 0.1f, 1.0f, "%.3f");
                ImGui::InputFloat("b", &sprottB, 0.1f, 1.0f, "%.3f");
            }
        }

        if (ImGui::CollapsingHeader("Particles", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::InputInt("Number of particles", &numParticles, 1, 1);
            if(ImGui::Button("Add particles")){
                particles=createParticles(numParticles,numPoints);
                cout<<particles.size()<<endl;
                //drawParticles();
            }
        }

        if (ImGui::Button("Generate Trajectory")) {
            trajectory = generate_trajectory(currentAttractor, initialPoint,numPoints,maxTime);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, trajectory.size() * sizeof(glm::vec3), trajectory.data(), GL_STATIC_DRAW);
        }
        if (ImGui::Button("Generate Trajectory CSV")) {
            x=initialPoint.x;
            y=initialPoint.y;
            z=initialPoint.z;
            Point pt(x,y,z);
            vector<vector<float>>csv=generate_trajectory_csv(currentAttractor,pt,numPoints,maxTime);
        }
        if (ImGui::Button("Reset View")) {
            // Reset camera logic
            yaw = -90.0f;
            pitch = 0.0f;
            radius = 50.0f;
            cameraTarget = glm::vec3(0.0f);
        }
        ImGui::End();
        // =====================

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        //Camera
        glm::vec3 direction;
        direction.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
        direction.y = sin(glm::radians(pitch));
        direction.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));

        glm::vec3 cameraPos = cameraTarget + cameraDistance * direction;
        glm::mat4 view = glm::lookAt(cameraPos, cameraTarget, glm::vec3(0.0f, 1.0f, 0.0f));

        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);
        glm::mat4 mvp = projection * view * model;

        glUseProgram(shaderProgram);
        GLint mvpLoc = glGetUniformLocation(shaderProgram, "mvp");
        glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, glm::value_ptr(mvp));

        // Update particles
        for (auto& p : particles) {
            p.position +=p.speed*dt; // change that later

            // Loop back to start
            if (p.position >= trajectory.size() - 1) {
                p.position = 0.0f;
            }
        }

        //Draw particles
        glBindVertexArray(VAO);
        glDrawArrays(GL_LINE_STRIP, 0, trajectory.size());
        glBindVertexArray(0);

        // Interpolate particle positions
        particlePositions.clear();
        particleColors.clear();

        for (auto& p : particles) {
            int idx = static_cast<int>(p.position);   // base index
            float alpha = p.position - idx;           // fractional part
            float t = alpha * dt;
            glm::vec3 pos = getPositionAt(trajectory[idx],t,currentAttractor);
            particlePositions.push_back(pos);
            particleColors.push_back(p.color);
        }

        // Upload to VBO:
        glBindBuffer(GL_ARRAY_BUFFER, particlePosVBO);
        glBufferData(GL_ARRAY_BUFFER, particlePositions.size() * sizeof(glm::vec3), particlePositions.data(), GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, particleColorVBO);
        glBufferData(GL_ARRAY_BUFFER, particleColors.size() * sizeof(glm::vec3), particleColors.data(), GL_DYNAMIC_DRAW);

        // Draw particles
        glBindVertexArray(particleVAO);
        glDrawArrays(GL_POINTS, 0, particlePositions.size());
        glBindVertexArray(0);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        cerr << "OpenGL error: " << err << endl;
    }

    // Cleanup
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();


    glfwTerminate();
    return 0;
}

/*int main() {
    if (!glfwInit()) {
        cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(800, 600, "Triangle Test", nullptr, nullptr);
    if (!window) {
        cerr << "Failed to create window\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        cerr << "Failed to initialize GLAD\n";
        return -1;
    }

    glEnable(GL_DEPTH_TEST);
    glLineWidth(2.0f);

    // Define triangle vertices
    glm::vec3 triangle[] = {
        glm::vec3(-1.0f, -1.0f, 0.0f),
        glm::vec3( 1.0f, -1.0f, 0.0f),
        glm::vec3( 0.0f,  1.0f, 0.0f)
    };

    GLuint VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(triangle), triangle, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    GLuint shaderProgram = LoadShaders("../shaders/vertex.glsl", "../shaders/fragment.glsl");
    glUseProgram(shaderProgram);
    cout << "Shader program ID: " << shaderProgram << endl;

    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::lookAt(glm::vec3(0, 0, 5), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);
        glm::mat4 mvp = projection * view * model;

        glUseProgram(shaderProgram);
        GLint mvpLoc = glGetUniformLocation(shaderProgram, "mvp");
        glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, glm::value_ptr(mvp));

        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        cerr << "OpenGL error: " << err << endl;
    }

    // Cleanup
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);

    glfwTerminate();
    return 0;
}*/

