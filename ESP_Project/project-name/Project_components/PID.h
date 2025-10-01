#ifndef PID_H
#define PID_H
#include <math.h>

struct PID{
    float Kp;
    float Ki;
    float Kd;
    float error; 
    float previous_error;
    float clearance;
    float integral;
    float filter;
    float alpha; // filter coefficient
    float output;
    float output_min; 
    float output_max; 
};
void pid_init(struct PID* pid, float Kp, float Ki, float Kd,float clearance, float output_min, float output_max);
void pid_compute(struct PID* pid, float error, float dt);
#endif // PID_H