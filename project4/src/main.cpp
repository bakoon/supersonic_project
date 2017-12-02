//state definition
#define INIT 0
#define PATH_PLANNING 1
#define RUNNING 2
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <pwd.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

//map spec
cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

//parameters we should adjust : K, margin, MaxStep
int margin = 5;
int K = 500;
double MaxStep = 2;
int waypoint_margin = 22;

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;

//FSM state
int state;

//function definition
void setcmdvel(double v, double w);
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void set_waypoints();
void generate_path_RRT();

int main(int argc, char** argv){
    ros::init(argc, argv, "slam_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",1);
    
    ros::Subscriber gazebo_pose_sub = n.subscribe("/amcl_pose", 100, callback_state);

    printf("Initialize topics\n");
    
    // FSM
    state = INIT;
    bool running = true;
    ros::Rate control_rate(60);

    while(running){
        switch (state) {
        case INIT: {
            
            // Load Map
            char* user = getpwuid(getuid())->pw_name;
            cv::Mat map_org = cv::imread((std::string("/home/") + std::string(user) +
                              std::string("/catkin_ws/src/project4/src/slam_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.5;
            world_x_max = 4.5;
            world_y_min = -13.5;
            world_y_max = 13.5;
            res = 0.05;
            printf("Load map\n");

             if(! map.data )                              // Check for invalid input
            {
                printf("Could not open or find the image\n");
                return -1;
            }
            state = PATH_PLANNING;
        } break;

        case PATH_PLANNING:
            
            // Set Way Points
            set_waypoints();
            printf("Set way points\n");

            // RRT
            generate_path_RRT();
            printf("Generate RRT\n");

            ros::spinOnce();
            ros::Rate(0.33).sleep();

            printf("Initialize ROBOT\n");
            state = RUNNING;

        case RUNNING: {
            //TODO 1
            int look_ahead_idx = 0;
            PID pid_ctrl;
            const double max_ctrl = 0.8;
            while(ros::ok()) {
                traj cur_goal = path_RRT[look_ahead_idx];
                double ctrl = pid_ctrl.get_control(robot_pose, cur_goal, cur_goal);
                if (ctrl > 0) {
                    ctrl = MIN(max_ctrl, ctrl);
                }
                else {
                    ctrl = MAX(-max_ctrl, ctrl);
                }
                //printf("car pose %.3f, %.3f, cur goal %.3f, %.3f, ctrl %.3f\n", robot_pose.x, robot_pose.y, cur_goal.x, cur_goal.y, ctrl);
                cmd.drive.speed = 1.5;
                cmd.drive.steering_angle = ctrl; //pid~'
                cmd_vel_pub.publish(cmd);
                // TO DO
                const double distance_check = 0.3;
                double x_err = robot_pose.x - cur_goal.x;
                double y_err = robot_pose.y - cur_goal.y;
                double distance = x_err * x_err + y_err * y_err;
                //printf("distance %.3f\n", distance);
                if (distance < distance_check * distance_check) {
                    //printf("arrived %d, distance: %.3f\n", look_ahead_idx, std::sqrt(distance));
                    look_ahead_idx++;
                    pid_ctrl.reset();
                }
                if (look_ahead_idx == path_RRT.size()) {
                    state = FINISH;
                    //printf("Goal in!!, state: %d\n", state );
                    break;
                }

                ros::spinOnce();
                control_rate.sleep();
		  	}
        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd);
            running = false;
            ros::spinOnce();
            control_rate.sleep();
        } break;
        default: {
        } break;
        }
    }
    return 0;
}

void setcmdvel(double vel, double deg){
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}

void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs){
    robot_pose.x = msgs->pose.pose.position.x;
    robot_pose.y = msgs->pose.pose.position.y;
    robot_pose.th = tf::getYaw(msgs->pose.pose.orientation);
    //printf("x,y : %f,%f \n",robot_pose.x,robot_pose.y);
}

void set_waypoints()
{
    std::srand(std::time(NULL));
    point waypoint_candid[5];
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 12.0;

    cv::Mat map_margin = map.clone();
    int jSize = map.cols; // the number of columns
    int iSize = map.rows; // the number of rows

    for (int i = 0; i < iSize; i++) {
        for (int j = 0; j < jSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - waypoint_margin; k <= i + waypoint_margin; k++) {
                    for (int l = j - waypoint_margin; l <= j + waypoint_margin; l++) {
                        if (k >= 0 && l >= 0 && k < iSize && l < jSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    //TODO 2
    // Make your own code to select waypoints.
    // You can randomly sample some points from the map.
    // Also, the car should follow the track in clockwise.

    double world_x[4];
    double world_y[4];

    world_x[0] = world_x_min;
    world_y[0] = world_y_max;
    world_x[1] = world_x_max;
    world_y[1] = world_y_max;
    world_x[2] = world_x_max;
    world_y[2] = world_y_min;
    world_x[3] = world_x_min;
    world_y[3] = world_y_min;

    //res = 0.05;
    int i;
    int j;
    for (int k=1; k<4; k++) {
        int collision = 0;
        while(collision != 250){
            waypoint_candid[k].x = world_x[k] * (double)rand() / (double)RAND_MAX;
    		waypoint_candid[k].y = world_y[k] * (double)rand() / (double)RAND_MAX;
			i = int(waypoint_candid[k].x / res + map_origin_x);
            j = int(waypoint_candid[k].y / res + map_origin_y);
            collision = map_margin.at<uchar>(i,j);
        }
    }

    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 12.0;

    int order[] = {0,1,2,3,4};
    int order_size = 5;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void generate_path_RRT()
{
    //TODO 1
    srand((int)time(NULL));
    traj start_traj;
    start_traj.x = waypoints[0].x;
    start_traj.y = waypoints[0].y;
    start_traj.th = waypoints[0].th;
    path_RRT.push_back(start_traj);

    printf("waypoint size : %d\n", waypoints.size());
    point x_init = waypoints[0];
    //printf("xinit x %.3f, y %.3f\n", x_init.x, x_init.y);
    point x_goal;
    rrtTree thisTree;
    traj lastPoint;
    std::vector<traj> traj_array[waypoints.size()];
    int goal_idx = 1;

    //for (int i = 0; i < waypoints.size()-1; i++) {
    //for (int i = 1; i < waypoints.size(); i++) {
    while( goal_idx < waypoints.size() ) {
        x_goal = waypoints[goal_idx];
        printf("generateRRT %d / %d\n", goal_idx, waypoints.size()-1);
        int ret_gen = 1;
        int ret_count = 0;
        while(ret_gen) {
            if (goal_idx != 1 && ret_count == 2) { // failed twice : retry previous one
                break;
            }
            thisTree = rrtTree(x_init, x_goal, map, map_origin_x, map_origin_y, res, margin);
            ret_gen = thisTree.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
            ret_count++;
        }

        if (ret_gen) { // come back
            if (goal_idx == 2) {
                x_init = waypoints[0];

            }
            else {
                lastPoint = traj_array[goal_idx-3][0];
                x_init.x = lastPoint.x;
                x_init.y = lastPoint.y;
                x_init.th = lastPoint.th;
            }
            goal_idx--;
        }
        else { // go next
            traj_array[goal_idx-1] = thisTree.backtracking_traj();
            lastPoint = traj_array[goal_idx-1][0];
            x_init.x = lastPoint.x;
            x_init.y = lastPoint.y;
            x_init.th = lastPoint.th;
            goal_idx++;
        }
    }
    for (int i = 0; i < waypoints.size(); i++) {
        std::vector<traj> this_traj = traj_array[i];
        for (int j = this_traj.size()-1; j >= 0; j--) {
            path_RRT.push_back(this_traj[j]);
        }
    }
    thisTree.visualizeTree(path_RRT);

}