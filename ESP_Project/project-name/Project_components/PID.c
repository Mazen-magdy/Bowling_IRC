#include "./PID.h"
void pid_init(struct PID* pid, float Kp, float Ki, float Kd,float clearance, float output_min, float output_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->error = 0.0f;
    pid->clearance = clearance;
    pid->output_min = output_min;
    pid->output_max = output_max;
}

void pid_compute(struct PID* pid, float error, float dt) {
    pid->error = error;
    if (fabs(pid->error) < pid->clearance) {
        pid->error = 0.0f; // Dead zone
    }
    // Proportional term
    float Pout = pid->Kp * pid->error;

    // Integral term with anti-windup
    pid->integral += (pid->error + pid->previous_error) / 2 * dt;
    float Iout = pid->Ki * pid->integral;

    // Derivative term
    float derivative = pid->alpha * (pid->error - pid->previous_error) / dt + (1 - pid->alpha) * pid->filter;
    float Dout = pid->Kd * derivative;
    pid->filter = derivative; // Update filter state

    // Compute total output
    pid->output = Pout + Iout + Dout;
    
    // Apply output limits
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
        // Anti-windup: prevent integral from increasing further
        if (pid->error > 0) {
            pid->integral -= pid->error * dt; // Undo last integral addition
        }
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
        // Anti-windup: prevent integral from decreasing further
        if (pid->error < 0) {
            pid->integral -= pid->error * dt; // Undo last integral addition
        }
    }

    // Update previous error
    pid->previous_error = pid->error;
}


/*
    timer interrupt every 10ms
        setFlag = true;
    main loop
        if(setFlag){
            pid_compute(&pid, setpoint, measured_value, 0.01f);
            combine_pid_outputs(&pid);
            motor_set_speed(&motor, pid.output);
            setFlag = false;
        }
*/