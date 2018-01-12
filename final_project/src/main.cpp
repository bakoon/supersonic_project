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
int margin = 4;
int K = 200;
double MaxStep = 2;
int waypoint_margin = 20;

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
                              std::string("/catkin_ws/src/final_project/src/final.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.7;
            world_x_max = 4.7;
            world_y_min = -10.2;
            world_y_max = 10.2;
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
            const double max_ctrl = 0.6;
            double speed;
            while(ros::ok()) {
                traj cur_goal = path_RRT[look_ahead_idx];
                double x_err = robot_pose.x - cur_goal.x;
                double y_err = robot_pose.y - cur_goal.y;
                double distance = x_err * x_err + y_err * y_err;
                double ctrl = pid_ctrl.get_control(robot_pose, cur_goal, cur_goal);
                double new_speed;
                if (ctrl > 0) {
                    ctrl = MIN(max_ctrl, ctrl);
                }
                else {
                    ctrl = MAX(-max_ctrl, ctrl);
                }
                //printf("car pose %.3f, %.3f, cur goal %.3f, %.3f, ctrl %.3f\n", robot_pose.x, robot_pose.y, cur_goal.x, cur_goal.y, ctrl);
                /*
                new_speed = MIN(std::exp(distance/2), 3.5-std::exp(std::abs(ctrl)));
                new_speed = MAX(new_speed, 1.6);
                double diff = new_speed - speed;
                if (diff < 0) { //brake
                    new_speed = (speed + new_speed) / 2.;
                }
                else { //accelerate
                    new_speed = speed + 0.3 * (new_speed - speed);
                }
                speed = new_speed;
                cmd.drive.speed = speed;
                */
                if (std::abs(ctrl) < 0.1)
                {
                    ctrl = 0;
                    new_speed = 2.5;
                }
                else if (std::abs(ctrl) == max_ctrl)
                {
                    new_speed = 1.5;
                }
                else
                {
                    new_speed = 2.0;
                }
                //printf("ctrl %.2f, distance %.2f, newspeed %.2f\n", ctrl, distance, new_speed);
                cmd.drive.speed = new_speed;
                cmd.drive.steering_angle = ctrl; //pid~'
                cmd_vel_pub.publish(cmd);
                // TO DO
                const double distance_check = 0.6;
                //printf("distance %.3f\n", distance);
                if (distance < distance_check) {
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
    point waypoint_candid[7];

    // Starting point. (Fixed)
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 8.5;

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
    // Set your own waypoints.
    // The car should turn around the outer track once, and come back to the starting point.
    // This is an example.
    double world_x[9];
    double world_y[9];

    world_x[0] = world_x_min;
    world_y[0] = world_y_max;
    world_x[1] = world_x_max;
    world_y[1] = world_y_max;
    world_x[2] = world_x_max;
    world_y[2] = world_y_min;
    world_x[3] = world_x_min;
    world_y[3] = world_y_min;

    //res = 0.05;
    const int num_iter = 100;
    int i;
    int j;
    for (int k=1; k<4; k++) {
        double max_distance = 0;
        int collision = 0;
        for (int l=0; l<num_iter; l++) {
            int rand_i = world_x[k%4] * (double)rand() / (double)RAND_MAX;
            int rand_j = world_y[k%4] * (double)rand() / (double)RAND_MAX;
            i = int(rand_i / res + map_origin_x);
            j = int(rand_j / res + map_origin_y);
            collision = map_margin.at<uchar>(i,j);
            //printf("collision : %d\n", collision);
            if (collision == 254) {
                double distance = rand_i * rand_i + rand_j * rand_j;
                if (distance > max_distance){
                    max_distance = distance;

                    //printf("x %.2f, y %.2f\n", rand_i, rand_j);
                    waypoint_candid[k].x = rand_i;
                    waypoint_candid[k].y = rand_j;
                }
            }
        }
    }


    waypoint_candid[4].x = waypoint_candid[0].x;
    waypoint_candid[4].y = waypoint_candid[0].y;


    // Waypoints for arbitrary goal points.
    // TA will change this part before scoring.
    // This is an example.
    waypoint_candid[5].x = 1.5;
    waypoint_candid[5].y = 1.5;
    waypoint_candid[6].x = 1.5;
    waypoint_candid[6].y = 3;
    //waypoint_candid[6].x = -2;
    //waypoint_candid[6].y = -9.0;

    int order[] = {0,1,2,3,4,5,6};
    int order_size = 7;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}


void visualize_multiple_images(std::vector<cv::Mat> images) {
    int w = 4;
    int h = 2;
    int size_x = 400;
    int size_y = 200;
    int i, m, n;
    cv::Mat dispimage = cv::Mat::zeros(cv::Size(100 + size_x*w, 60 + size_y*h), CV_8UC3);
    
    for ( i = 0, m = 20, n = 20; i < images.size(); i++, m+= (20+size_x)) {
        int x = images[i].cols;
        int y = images[i].rows;

        float scale = (float) ( (float) x / size_x);

        if (i % w == 0 && m!= 20){
            m = 20;
            n+= 20 + size_y;
        }

        cv::Rect ROI(m, n, (int)( x/scale ), (int)( y/scale ));
        cv::Mat temp;
        cv::resize(images[i], temp, cv::Size(ROI.width, ROI.height));
        temp.copyTo(dispimage(ROI));
    }

    cv::namedWindow( "Mapping", 1 );
    cv::imshow( "Mapping", dispimage );
    cv::waitKey();

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
    std::vector<cv::Mat> images;
    for (int i = 0; i < waypoints.size(); i++) {
        std::vector<traj> this_traj = traj_array[i];
        for (int j = this_traj.size()-1; j >= 0; j--) {
            path_RRT.push_back(this_traj[j]);
        }
        images.push_back(thisTree.visualizeTree(path_RRT));
    }
    //visualize_multiple_images(images);
}


