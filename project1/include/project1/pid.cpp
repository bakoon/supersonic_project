#include <project1/pid.h>
#include <stdio.h>
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))  
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))  

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


float PID::get_control(point car_pose, point goal_pose){

    float weight_g, weight_d;
   
    float distance;

    float ctrl;
    float x_mv, y_mv;
    float x_err, y_err, th_goal;
    float th_err;
    float error_term, integral_term, derivative_term;

    float angle, v_head_x, v_head_y, v_goal_x, v_goal_y;
    float angle_head, angle_goal;

    //calculate th_goal
    
    x_err = goal_pose.x - car_pose.x;
    y_err = goal_pose.y - car_pose.y; 
    th_goal = atan(y_err/ x_err);

    distance = sqrt(x_err*x_err + y_err*y_err);
    
    //weight_g = MIN(distance, 2.0)/2.2;
    //weight_d = 1 - weight_g;
    weight_g = 0.95;
    weight_d = 0.05;
    
    if (x_err < 0 ) {
        if (y_err > 0) {
            th_goal += M_PI;
        }
        if (y_err < 0) {
            th_goal -= M_PI;
        }
    }

    //calculate th_error
    th_err = weight_g * th_goal + weight_d * goal_pose.th - car_pose.th;
    th_err = fmod(th_err, 2*M_PI);
    if (th_err > M_PI) {
        th_err -= 2*M_PI;
    }
    else if (th_err < -M_PI) {
        th_err += 2*M_PI;
    }


    //calculate ctrl
    error_term = this->Kp * th_err;
    integral_term = this->Ki * this->delta_t * (this->error_sum);// + th_err);
    derivative_term = (this->Kd / this->delta_t) * (th_err - this->error);
    
    ctrl = error_term + integral_term + derivative_term; 
    //ctrl = fmod(ctrl, 2*M_PI);
    //update values
    this->error = th_err;
    this->error_sum *= 0.9;
    this->error_sum += th_err;

    printf("theta_goal : %.2f, theta error : %.2f, ctrl : %.2f\n", th_goal, th_err, ctrl);
    
    return ctrl;
    
}
