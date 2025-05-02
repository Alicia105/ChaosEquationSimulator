#include <iostream>
#include <fstream>
#include <vector>
#include <bits/stdc++.h>
#include "../include/equations.hpp"

//#include <SFML/Graphics.hpp>
using namespace std;
struct Point{
    float x;
    float y;
    float z;    
};

/* Eq de Lorenz

(dx/dt)= sigma*(py-px)
(dy/dt)=px*(rho-pz)-py
(dz/dt)=px*py-beta*pz
*/

/* Valeurs caractéristiques :
sigma =10
rho=28
beta=8/3
*/

vector<float> lorenz(Point point,float dt, float sigma, float rho, float beta){
       
    float x=point.x;
    float y=point.y;
    float z=point.z;

    float dx = sigma*(y-x)*dt;
    float dy = (x*(rho-z)-y)*dt;
    float dz = (x*y-beta*z)*dt;

    vector<float> result={dt,dx,dy,dz};

    return result;
}

vector<vector<float>> lorenz_trajectory(Point initialPoint,int numPoints,float maxTime, float sigma, float rho, float beta){

    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);

    char output[50];
  
    strftime(output, 50, "%e-%b-%Y_%H-%M-%S ", &datetime);
    cout << output <<endl;
    string name = "../results/lorenz_" + string(output) + ".csv";

    string const file(name);

    ofstream stream(file.c_str());

    bool writting;

    if(stream){
        writting=true;
        cout << "File opened: " << name << std::endl;        
    }
    else{
        writting=false;   
        cout << "ERREUR: Impossible d'ouvrir le fichier." << endl;
    }

    vector<Point> trajectory;
    vector<float>  timeStamp;
    float dt=maxTime/numPoints;

    vector<vector<float>> result;

    trajectory.push_back(initialPoint);
    timeStamp.push_back(0.0);
    
    vector<float> startingPoint={0.0,initialPoint.x,initialPoint.y,initialPoint.z};
    result.push_back(startingPoint);
    // Example write (optional)
    if (writting) {
        stream << "time,x,y,z"<<endl; // CSV header
        stream<<startingPoint[0]<<","<<startingPoint[1]<<","<<startingPoint[2]<<","<<startingPoint[3]<<endl;
    }   

    for(int i=0; i<numPoints;i++){
        vector<float> r=(lorenz(trajectory[i],dt,sigma,rho,beta));
        r[0]=timeStamp[i]+dt;

        Point p;
        p.x=r[1];
        p.y=r[2];
        p.z=r[3];

        trajectory.push_back(p);
        timeStamp.push_back(r[0]);
        result.push_back(r);

        if(writting){
            stream<<r[0]<<","<<r[1]<<","<<r[2]<<","<<r[3]<<endl;
        }

    }


    stream.close();

    return result;
}

/*Equation de Rossler:
dx/dt=-y-z
dy/dt=x+ay
dz/dt=b+z(x-c)
*/

/*Valeurs de a, b, c possibles:
a=0.2 ou 0.1
b=0.2 ou 0.1
c=5.7 ou c=14
*/

vector<float> rossler(Point point,float dt, float a, float b, float c){

    float x=point.x;
    float y=point.y;
    float z=point.z;

    float dx = (-(y-z))*dt;
    float dy = (x+a*z)*dt;
    float dz = (b+z*(x-c))*dt;

    vector<float> result={dt,dx,dy,dz};

    return result;
}

vector<vector<float>> rossler_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b, float c){

    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);

    char output[50];
  
    strftime(output, 50, "%e-%b-%Y_%H-%M-%S ", &datetime);
    cout << output <<endl;
    string name = "../results/rossler_" + string(output) + ".csv";

    string const file(name);

    ofstream stream(file.c_str());

    bool writting;

    if(stream){
        writting=true;
        cout << "File opened: " << name << std::endl;        
    }
    else{
        writting=false;   
        cout << "ERREUR: Impossible d'ouvrir le fichier." << endl;
    }

    vector<Point> trajectory;
    vector<float>  timeStamp;
    float dt=maxTime/numPoints;

    vector<vector<float>> result;

    trajectory.push_back(initialPoint);
    timeStamp.push_back(0.0);
    
    vector<float> startingPoint={0.0,initialPoint.x,initialPoint.y,initialPoint.z};
    result.push_back(startingPoint);
    // Example write (optional)
    if (writting) {
        stream << "time,x,y,z"<<endl; // CSV header
        stream<<startingPoint[0]<<","<<startingPoint[1]<<","<<startingPoint[2]<<","<<startingPoint[3]<<endl;
    }    

    for(int i=0; i<numPoints;i++){
        vector<float> r=(rossler(trajectory[i],dt,a,b,c));
        r[0]=timeStamp[i]+dt;

        Point p;
        p.x=r[1];
        p.y=r[2];
        p.z=r[3];
        
        trajectory.push_back(p);
        timeStamp.push_back(r[0]);
        result.push_back(r);

        if(writting){
            stream<<r[0]<<","<<r[1]<<","<<r[2]<<","<<r[3]<<endl;
        }
    }

    stream.close();
    return result;
}

/*Equations de Li:
dx/dt=a*(y-x)+d*x*z
dy/dt=k*x+f*y-x*z
dz/dt=c*z+x*y-e*x^2
*/

/*Valeurs de a,b,c,d,e,f:
a=41
b=11 ou 1.833
c=0.16
d=0.65
e=55
f=20
*/

vector<float> dequan_li(Point point,float dt,float a, float b, float c, float d, float e, float f){
       
    float x=point.x;
    float y=point.y;
    float z=point.z;

    float dx = (a*(y-x)+c*x*z)*dt;
    float dy = (e*x+f*y-x*z)*dt;
    float dz = (b*z+x*y-d*pow(x,2))*dt;

    vector<float> result={dt,dx,dy,dz};

    return result;
}

vector<vector<float>> dequan_li_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b, float c, float d, float e, float f){
    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);

    char output[50];
  
    strftime(output, 50, "%e-%b-%Y_%H-%M-%S ", &datetime);
    cout << output <<endl;
    string name = "../results/dequan_li_" + string(output) + ".csv";

    string const file(name);

    ofstream stream(file.c_str());

    bool writting;

    if(stream){
        writting=true;
        cout << "File opened: " << name << std::endl;        
    }
    else{
        writting=false;   
        cout << "ERREUR: Impossible d'ouvrir le fichier." << endl;
    }

    vector<Point> trajectory;
    vector<float>  timeStamp;
    float dt=maxTime/numPoints;

    vector<vector<float>> result;

    trajectory.push_back(initialPoint);
    timeStamp.push_back(0.0);
    
    vector<float> startingPoint={0.0,initialPoint.x,initialPoint.y,initialPoint.z};
    result.push_back(startingPoint);
    // Example write (optional)
    if (writting) {
        stream << "time,x,y,z"<<endl; // CSV header
        stream<<startingPoint[0]<<","<<startingPoint[1]<<","<<startingPoint[2]<<","<<startingPoint[3]<<endl;
    }    

    for(int i=0; i<numPoints;i++){
        vector<float> r=(dequan_li(trajectory[i],dt,a,b,c,d,e,f));
        r[0]=timeStamp[i]+dt;

        Point p;
        p.x=r[1];
        p.y=r[2];
        p.z=r[3];
        
        trajectory.push_back(p);
        timeStamp.push_back(r[0]);
        result.push_back(r);

        if(writting){
            stream<<r[0]<<","<<r[1]<<","<<r[2]<<","<<r[3]<<endl;
        }
    
    }

    stream.close();
    return result;
}

/*Equations de Aizawa:
dx/dt=(z-b)*x-d*y
dy/dt=d*x+(z-b)*y
dz/dt=c+a*z-pow(z,3)/3-pow(x,2)+f*z*pow(x,3)
*/

/*Valeurs de a,b,c,d,e,f:
a=0.95
b=0.7
c=0.6
d=3.5
e=0.25
f=0.1
*/

vector<float> aizawa(Point point,float dt,float a, float b, float c, float d, float e, float f){
       
    float x=point.x;
    float y=point.y;
    float z=point.z;

    float dx=((z-b)*x-d*y)*dt;
    float dy=(d*x+(z-b)*y)*dt;
    float dz=(c+a*z-pow(z,3)/3-pow(x,2)+f*z*pow(x,3));

    vector<float> result={dt,dx,dy,dz};

    return result;
}

vector<vector<float>> aizawa_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b, float c, float d, float e, float f){

    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);

    char output[50];
  
    strftime(output, 50, "%e-%b-%Y_%H-%M-%S ", &datetime);
    cout << output <<endl;
    string name = "../results/aizawa_" + string(output) + ".csv";

    string const file(name);

    ofstream stream(file.c_str());

    bool writting;

    if(stream){
        writting=true;
        cout << "File opened: " << name << std::endl;        
    }
    else{
        writting=false;   
        cout << "ERREUR: Impossible d'ouvrir le fichier." << endl;
    }

    vector<Point> trajectory;
    vector<float>  timeStamp;
    float dt=maxTime/numPoints;

    vector<vector<float>> result;

    trajectory.push_back(initialPoint);
    timeStamp.push_back(0.0);
    
    vector<float> startingPoint={0.0,initialPoint.x,initialPoint.y,initialPoint.z};
    result.push_back(startingPoint);

    // Example write (optional)
    if (writting) {
        stream << "time,x,y,z"<<endl; // CSV header
        stream<<startingPoint[0]<<","<<startingPoint[1]<<","<<startingPoint[2]<<","<<startingPoint[3]<<endl;
    }    

    for(int i=0; i<numPoints;i++){
        vector<float> r=(aizawa(trajectory[i],dt,a,b,c,d,e,f));
        r[0]=timeStamp[i]+dt;

        Point p;
        p.x=r[1];
        p.y=r[2];
        p.z=r[3];
        
        trajectory.push_back(p);
        timeStamp.push_back(r[0]);
        result.push_back(r);

        if(writting){
            stream<<r[0]<<","<<r[1]<<","<<r[2]<<","<<r[3]<<endl;
        }
      
    }

    stream.close();
    return result;
}

/*Equations de Chen-Lee:
dx/dt=a*x-y*z
dy/dt=b*y+x*z
dz/dt=d*z+(x*y)/3
*/

/*Valeurs de a,b,d:
a=5
b=-10
d=-0.38
*/

vector<float> chen_lee(Point point,float dt,float a, float b, float d){
       
    float x=point.x;
    float y=point.y;
    float z=point.z;

    float dx=(a*x-y*z)*dt;
    float dy=(b*y+x*z)*dt;
    float dz=(d*z+(x*y)/3)*dt;

    vector<float> result={dt,dx,dy,dz};

    return result;
}

vector<vector<float>> chen_lee_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b,float d){
    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);

    char output[50];
  
    strftime(output, 50, "%e-%b-%Y_%H-%M-%S ", &datetime);
    cout << output <<endl;
    string name = "../results/chen_lee_" + string(output) + ".csv";

    string const file(name);

    ofstream stream(file.c_str());

    bool writting;

    if(stream){
        writting=true;
        cout << "File opened: " << name << std::endl;        
    }
    else{
        writting=false;   
        cout << "ERREUR: Impossible d'ouvrir le fichier." << endl;
    }

    vector<Point> trajectory;
    vector<float>  timeStamp;
    float dt=maxTime/numPoints;

    vector<vector<float>> result;

    trajectory.push_back(initialPoint);
    timeStamp.push_back(0.0);
    
    vector<float> startingPoint={0.0,initialPoint.x,initialPoint.y,initialPoint.z};
    result.push_back(startingPoint);
    
    // Example write (optional)
    if (writting) {
        stream << "time,x,y,z"<<endl; // CSV header
        stream<<startingPoint[0]<<","<<startingPoint[1]<<","<<startingPoint[2]<<","<<startingPoint[3]<<endl;
    }

    for(int i=0; i<numPoints;i++){
        vector<float> r=(chen_lee(trajectory[i],dt,a,b,d));
        r[0]=timeStamp[i]+dt;

        Point p;
        p.x=r[1];
        p.y=r[2];
        p.z=r[3];
        
        trajectory.push_back(p);
        timeStamp.push_back(r[0]);
        result.push_back(r);

        if(writting){
            stream<<r[0]<<","<<r[1]<<","<<r[2]<<","<<r[3]<<endl;
        }
    }

    stream.close();
    return result;
}

/*Equations de Arneodo:
dx/dt=y
dy/dt=z
dz/dt=-a*x-b*y-z+c*pow(x,3)
*/

/*Valeurs de a,b,c:
a=-5.5
b=3.5
c=-1
*/

vector<float> arneodo(Point point,float dt,float a, float b, float c){
       
    float x=point.x;
    float y=point.y;
    float z=point.z;

    float dx=y*dt;
    float dy=z*dt;
    float dz=(-a*x-b*y-z+c*pow(x,3))*dt;

    vector<float> result={dt,dx,dy,dz};

    return result;
}

vector<vector<float>>arneodo_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b,float c){

    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);

    char output[50];
  
    strftime(output, 50, "%e-%b-%Y_%H-%M-%S ", &datetime);
    cout << output <<endl;
    string name = "../results/arneodo_" + string(output) + ".csv";

    string const file(name);

    ofstream stream(file.c_str());

    bool writting;

    if(stream){
        writting=true;
        cout << "File opened: " << name << std::endl;        
    }
    else{
        writting=false;   
        cout << "ERREUR: Impossible d'ouvrir le fichier." << endl;
    }

    vector<Point> trajectory;
    vector<float>  timeStamp;
    float dt=maxTime/numPoints;

    vector<vector<float>> result;

    trajectory.push_back(initialPoint);
    timeStamp.push_back(0.0);
    
    vector<float> startingPoint={0.0,initialPoint.x,initialPoint.y,initialPoint.z};
    result.push_back(startingPoint);

    // Example write (optional)
    if (writting) {
        stream << "time,x,y,z"<<endl; // CSV header
        stream<<startingPoint[0]<<","<<startingPoint[1]<<","<<startingPoint[2]<<","<<startingPoint[3]<<endl;
    }

    for(int i=0; i<numPoints;i++){
        vector<float> r=(arneodo(trajectory[i],dt,a,b,c));
        r[0]=timeStamp[i]+dt;

        Point p;
        p.x=r[1];
        p.y=r[2];
        p.z=r[3];
        
        trajectory.push_back(p);
        timeStamp.push_back(r[0]);
        result.push_back(r);

        if(writting){
            stream<<r[0]<<","<<r[1]<<","<<r[2]<<","<<r[3]<<endl;
        }
    }
       
    stream.close();
    return result;
}

/*Equations de Sprott B:
dx/dt=a*y*z
dy/dt=x-b*y
dz/dt=c-x*y
*/

/*Valeurs de a,b,c:
a=0.4
b=1.2
c=1
*/

vector<float> sprott_b(Point point,float dt,float a, float b, float c){
       
    float x=point.x;
    float y=point.y;
    float z=point.z;

    float dx=a*y*z*dt;
    float dy=(x-b*y)*dt;
    float dz=(c-x*y)*dt;

    vector<float> result={dt,dx,dy,dz};

    return result;
}

vector<vector<float>> sprott_b_trajectory(Point initialPoint,int numPoints,float maxTime, float a, float b,float c){
    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);

    char output[50];
  
    strftime(output, 50, "%e-%b-%Y_%H-%M-%S ", &datetime);
    cout << output <<endl;
    string name = "../results/sprott_b_" + string(output) + ".csv";

    string const file(name);

    ofstream stream(file.c_str());

    bool writting;

    if(stream){
        writting=true;
        cout << "File opened: " << name << std::endl;        
    }
    else{
        writting=false;   
        cout << "ERREUR: Impossible d'ouvrir le fichier." << endl;
    }

    vector<Point> trajectory;
    vector<float>  timeStamp;
    float dt=maxTime/numPoints;

    vector<vector<float>> result;

    trajectory.push_back(initialPoint);
    timeStamp.push_back(0.0);
    
    vector<float> startingPoint={0.0,initialPoint.x,initialPoint.y,initialPoint.z};
    result.push_back(startingPoint);
    
    // Example write (optional)
    if (writting) {
        stream << "time,x,y,z"<<endl; // CSV header
        stream<<startingPoint[0]<<","<<startingPoint[1]<<","<<startingPoint[2]<<","<<startingPoint[3]<<endl;
    }

    for(int i=0; i<numPoints;i++){
        vector<float> r=(sprott_b(trajectory[i],dt,a,b,c));
        r[0]=timeStamp[i]+dt;

        Point p;
        p.x=r[1];
        p.y=r[2];
        p.z=r[3];
        
        trajectory.push_back(p);
        timeStamp.push_back(r[0]);
        result.push_back(r);

        if(writting){
            stream<<r[0]<<","<<r[1]<<","<<r[2]<<","<<r[3]<<endl;
        }
    
    }

    stream.close();
    return result;
}

/*Equations de Sprott-Linz F:
dx/dt=y+z
dy/dt=-x+a*y
dz/dt=pow(x,2)-z
*/

/*Valeurs de a:
a=0.5
*/

vector<float> sprott_linz_f(Point point,float dt,float a){
       
    float x=point.x;
    float y=point.y;
    float z=point.z;

    float dx=(y+z)*dt;
    float dy=(-x+a*y)*dt;
    float dz=(pow(x,2)-z)*dt;

    vector<float> result={dt,dx,dy,dz};

    return result;
}

vector<vector<float>> sprott_linz_f_trajectory(Point initialPoint,int numPoints,float maxTime, float a){
    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);

    char output[50];
  
    strftime(output, 50, "%e-%b-%Y_%H-%M-%S ", &datetime);
    cout << output <<endl;
    string name = "../results/sprott_linz_f_" + string(output) + ".csv";

    string const file(name);

    ofstream stream(file.c_str());

    bool writting;

    if(stream){
        writting=true;
        cout << "File opened: " << name << std::endl;        
    }
    else{
        writting=false;   
        cout << "ERREUR: Impossible d'ouvrir le fichier." << endl;
    }

    vector<Point> trajectory;
    vector<float>  timeStamp;
    float dt=maxTime/numPoints;

    vector<vector<float>> result;

    trajectory.push_back(initialPoint);
    timeStamp.push_back(0.0);
    
    vector<float> startingPoint={0.0,initialPoint.x,initialPoint.y,initialPoint.z};
    result.push_back(startingPoint);

    // Example write (optional)
    if (writting) {
        stream << "time,x,y,z"<<endl; // CSV header
        stream<<startingPoint[0]<<","<<startingPoint[1]<<","<<startingPoint[2]<<","<<startingPoint[3]<<endl;
    }

    for(int i=0; i<numPoints;i++){
        vector<float> r=(sprott_linz_f(trajectory[i],dt,a));
        r[0]=timeStamp[i]+dt;

        Point p;
        p.x=r[1];
        p.y=r[2];
        p.z=r[3];
        
        trajectory.push_back(p);
        timeStamp.push_back(r[0]);
        result.push_back(r);

        if(writting){
            stream<<r[0]<<","<<r[1]<<","<<r[2]<<","<<r[3]<<endl;
        }
    
    }

    stream.close();
    return result;
}

/*Equations de Dadras:
dx/dt=y-p*x+o*y*z
dy/dt=r*y-x*z+z
dz/dt=c*x*y-e*z
*/

/*Valeurs de p,o,r,c,e:
p=3
o=2.7
r=1.7
c=2
e=9
*/

vector<float> dadras(Point point,float dt,float p, float o,float r, float c, float e){
       
    float x=point.x;
    float y=point.y;
    float z=point.z;

    float dx=(y-p*x+o*y*z)*dt;
    float dy=(r*y-x*z+z)*dt;
    float dz=(c*x*y-e*z)*dt;

    vector<float> result={dt,dx,dy,dz};

    return result;
}

vector<vector<float>> dadras_trajectory(Point initialPoint,int numPoints,float maxTime, float p, float o,float r, float c, float e){
    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);

    char output[50];
  
    strftime(output, 50, "%e-%b-%Y_%H-%M-%S ", &datetime);
    cout << output <<endl;
    string name = "../results/dadras_" + string(output) + ".csv";

    string const file(name);

    ofstream stream(file.c_str());

    bool writting;

    if(stream){
        writting=true;
        cout << "File opened: " << name << std::endl;        
    }
    else{
        writting=false;   
        cout << "ERREUR: Impossible d'ouvrir le fichier." << endl;
    }

    vector<Point> trajectory;
    vector<float>  timeStamp;
    float dt=maxTime/numPoints;

    vector<vector<float>> result;

    trajectory.push_back(initialPoint);
    timeStamp.push_back(0.0);
    
    vector<float> startingPoint={0.0,initialPoint.x,initialPoint.y,initialPoint.z};
    result.push_back(startingPoint);
    
    // Example write (optional)
    if (writting) {
        stream << "time,x,y,z"<<endl; // CSV header
        stream<<startingPoint[0]<<","<<startingPoint[1]<<","<<startingPoint[2]<<","<<startingPoint[3]<<endl;
    }

    for(int i=0; i<numPoints;i++){
        vector<float> re=(dadras(trajectory[i],dt,p,o,r,c,e));
        re[0]=timeStamp[i]+dt;

        Point p;
        p.x=re[1];
        p.y=re[2];
        p.z=re[3];
        
        trajectory.push_back(p);
        timeStamp.push_back(re[0]);
        result.push_back(re);

        if(writting){
            stream<<re[0]<<","<<re[1]<<","<<re[2]<<","<<re[3]<<endl;
        }
    }

    stream.close();
    return result;
}

/*Equations de Halvorsen:
dx/dt=-a*x-4*y-4*z-y*y
dy/dt=-a*y-4*z-4*x-z*z
dz/dt=-a*z-4*x-4*y-x*x
*/

/*Valeurs de a:
a=1.4
*/

vector<float> halvorsen(Point point,float dt,float a){
       
    float x=point.x;
    float y=point.y;
    float z=point.z;

    float dx=(-a*x-4*y-4*z-y*y)*dt;
    float dy=(-a*y-4*z-4*x-z*z)*dt;
    float dz=(-a*z-4*x-4*y-x*x)*dt;

    vector<float> result={dt,dx,dy,dz};

    return result;
}

vector<vector<float>> halvorsen_trajectory(Point initialPoint,int numPoints,float maxTime, float a){
    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);

    char output[50];
  
    strftime(output, 50, "%e-%b-%Y_%H-%M-%S ", &datetime);
    cout << output <<endl;
    string name = "../results/halvorsen_" + string(output) + ".csv";

    string const file(name);

    ofstream stream(file.c_str());

    bool writting;

    if(stream){
        writting=true;
        cout << "File opened: " << name << std::endl;        
    }
    else{
        writting=false;   
        cout << "ERREUR: Impossible d'ouvrir le fichier." << endl;
    }

    vector<Point> trajectory;
    vector<float>  timeStamp;
    float dt=maxTime/numPoints;

    vector<vector<float>> result;

    trajectory.push_back(initialPoint);
    timeStamp.push_back(0.0);
    
    vector<float> startingPoint={0.0,initialPoint.x,initialPoint.y,initialPoint.z};
    result.push_back(startingPoint);    

    // Example write (optional)
    if (writting) {
        stream << "time,x,y,z"<<endl; // CSV header
        stream<<startingPoint[0]<<","<<startingPoint[1]<<","<<startingPoint[2]<<","<<startingPoint[3]<<endl;
    }

    for(int i=0; i<numPoints;i++){
        vector<float> r=(halvorsen(trajectory[i],dt,a));
        r[0]=timeStamp[i]+dt;

        Point p;
        p.x=r[1];
        p.y=r[2];
        p.z=r[3];
        
        trajectory.push_back(p);
        timeStamp.push_back(r[0]);
        result.push_back(r);

        if(writting){
            stream<<r[0]<<","<<r[1]<<","<<r[2]<<","<<r[3]<<endl;
        }
    }

    stream.close();
    return result;
}

