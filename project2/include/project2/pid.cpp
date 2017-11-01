#include <project2/pid.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846


PID::PID(){
    //initialize
    this->Kp = 0.015;
    this->Ki = 0.005;
    this->Kd = 0.080;
    this->delta_t = 0.1;
    this->error = 0;
    this->error_sum = 0;
}

void PID::reset(){
    this->error = 0;
    this->error_sum = 0;
}


/*
PID::PID(){

    error = 0;
    error_sum = 0;
    error_diff = 0;
    Kp = 1.5;
    Ki = 0;
    Kd = 5; 
}
*/
float PID::get_control(point car_pose, traj prev_goal, traj cur_goal) {
    //TODO
}

