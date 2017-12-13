#include <project2/pid.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846


PID::PID(){
    //initialize
    
    this->Kp = 1.5;
    this->Ki = 0.05;//0.05;
    this->Kd = 30./60.;//0.03;
    this->delta_t = 1. / 60.;
    this->error = 0;
    this->error_sum = 0;
    //this->error_diff = 0;
}

void PID::reset(){
    this->error = 0;
    this->error_sum = 0;
}
/*
float PID::get_control(point car_pose, traj prev_goal, traj cur_goal){
	float ctrl;

	float x_diff = cur_goal.x - car_pose.x;
	float y_diff = cur_goal.y - car_pose.y;
	float x_dot = cos(car_pose.th) * x_diff + sin(car_pose.th) * y_diff;
	float y_dot = -sin(car_pose.th) * x_diff + cos(car_pose.th) * y_diff;

	float angle_diff = fmod(cur_goal.th - car_pose.th, 2*PI);
	if(angle_diff > PI)angle_diff = angle_diff - 2*PI;
	
	float pid_angle = atan2(y_dot, x_dot);
	error_diff = pid_angle - error;
	error = pid_angle;
	error_sum += error;

	ctrl = Kp * error + Ki * error_sum + Kd * error_diff;

	if(ctrl>0.8){ctrl = 0.8;}
	else if(ctrl < -0.8){ctrl -= 0.8;}
	return ctrl;
}
*/
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
    ////why is there prev_goal?

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

    x_err = cur_goal.x - car_pose.x;
    y_err = cur_goal.y - car_pose.y;
    th_goal = atan(y_err/ x_err);

    distance = sqrt(x_err*x_err + y_err*y_err);

    weight_g = 1;
    weight_d = 0;

    if (x_err < 0 ) {
        if (y_err > 0) {
            th_goal += M_PI;
        }
        if (y_err < 0) {
            th_goal -= M_PI;
        }
    }

    //calculate th_error
    th_err = weight_g * th_goal + weight_d * cur_goal.th - car_pose.th;
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

    if(ctrl > 0.8 ){
	    ctrl = 0.8;
    }else if(ctrl < -0.8){
	    ctrl = -0.8;
    }

    //printf("----------------current status-----------------\n");
    //printf("car_pose: %.3f, %.3f, cur_goal: %.3f, %.3f\n", car_pose.x, car_pose.y, cur_goal.x, cur_goal.y);
    //printf("error: %.3f, integral: %.3f, derivative: %.3f, ctrl: %.3f\n", error_term, integral_term, derivative_term, ctrl);

    //update values
    this->error = th_err;
    this->error_sum *= 0.9;
    this->error_sum += th_err;

    //printf("errer term %.2f, integral term %.2f, derivative term %.2f\n car theta : %.2f, theta_goal : %.2f, theta error : %.2f, ctrl : %.2f\n", error_term, integral_term, derivative_term, car_pose.th, th_goal, th_err, ctrl);

    return ctrl;
}

