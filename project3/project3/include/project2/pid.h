#include <cmath>
#ifndef POINT_H
#define POINT_H
#include <project2/point.h>
#include <project2/traj.h>
#endif

class PID{
public:
    PID();

    //this function makes control output using arguments which are the current value and the target setpoint.
    void reset();
    float get_control(point car_pose, traj prev_goal, traj cur_goal);
private:
    float error;
    float error_sum;
    //float error_diff;
    float delta_t;
    float Kp;
    float Ki;
    float Kd;
};
