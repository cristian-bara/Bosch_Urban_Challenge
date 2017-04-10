#ifndef CAR_SYSTEM_H
#define CAR_SYSTEM_H

typedef struct{
    float x,y;
    float x_dot,y_dot;
    float x_dot_prev,y_dot_prev;
    float teta;
    float teta_dot;
    float omega;// Rotation speed in global coordinate system 
    float i;// Current
}TState;

typedef struct{
    // Motor input voltage;
    float voltage;
    // Steering angle
    float alpha;
}TInput;

#endif