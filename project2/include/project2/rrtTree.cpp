#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#define PI 3.14159265358979323846
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))  
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))  
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

double car_speed = 2.0;
double frequency = 60.0;

double max_alpha = 0.2;
double L = 0.325;

rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal) {
    this->x_init = x_init;
    this->x_goal = x_goal;

    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->alpha = 0;
    root->d = 0;
}

rrtTree::~rrtTree(){
    for (int i = 1; i <= count; i++) {
        delete ptrTable[i];
    }
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;
    std::srand(std::time(NULL));

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);
    
    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	for(int j = 0; j < 10; j++) {
	    double alpha = this->ptrTable[i]->alpha;
	    double d = this->ptrTable[i]->d;
	    double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	    double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	    double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	}
    }
    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::visualizeTree(std::vector<traj> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(path[0].y/res + map_origin_y)), (int)(Res*(path[0].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(path[path.size()-1].y/res + map_origin_y)), (int)(Res*(path[path.size()-1].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	for(int j = 0; j < 10; j++) {
	    double alpha = this->ptrTable[i]->alpha;
	    double d = this->ptrTable[i]->d;
	    double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	    double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	    double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	}
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
	for(int j = 0; j < 10; j++) {
	    double alpha = path[i].alpha;
	    double d = path[i].d;
	    double p1_th = path[i-1].th + d*j/10*tan(alpha)/L; // R = L/tan(alpha)
            double p2_th = path[i-1].th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = path[i-1].x + L/tan(alpha)*(sin(p1_th) - sin(path[i-1].th));
	    double p1_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p1_th));
            double p2_x = path[i-1].x + L/tan(alpha)*(sin(p2_th) - sin(path[i-1].th));
	    double p2_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	}
    }
    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha, double d) {
    //TODO
    node* newNode = new node;
    newNode->idx = this->count;
    this->ptrTable[count] = newNode;
    this->count += 1;
    newNode->rand = x_rand;
    newNode->location = x_new;
    newNode->idx_parent = idx_near;
    newNode->alpha = alpha;
    newNode->d = d;
}


int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    //TODO

    //values to be used as temp_variables.
    double out[5];
    double distance;
    int idx_near;
    int isNewState;
    point randVertex;
    point newVertex;

    //file to save the data
    //std::string file_name = "RRT_data.txt";
    //std::ofstream out_file(file_name.c_str());
    //out_file << "index\t" << "x\t y\t th\t alpha\t idx_parent\t distance" << std::endl;

    for (int i = 0; i < 20000; ++i) {
        randVertex = randomState(x_max, x_min, y_max, y_min);
	idx_near = nearestNeighbor(randVertex, MaxStep);
        if (idx_near != -1) {
	    //printf("idx_near: %d \n", idx_near);
            //printf("randVertex.x = %f, randVertex.y = %f\n", randVertex.x, randVertex.y);
	    isNewState = this->newState(out, ptrTable[idx_near]->location, randVertex, MaxStep);
	    //printf("isNewState: %d, !isNewState: %d", isNewState, !isNewState);
            if (!isNewState){
		//printf("isNewState: %d\n", isNewState);
                newVertex.x = out[0];
                newVertex.y = out[1];
                newVertex.th = out[2];
                addVertex(newVertex, randVertex, idx_near, out[3], out[4]);
                distance = std::sqrt((out[0] - x_goal.x)*(out[0] - x_goal.x) + (out[1] - x_goal.y)*(out[1] - x_goal.y));
		//printf("newVertex.point %f, %f, %f, alpha: %f, idx_parent: %d\n", newVertex.x, newVertex.y, newVertex.th,out[3], idx_near); 
                //printf("i: %d, distance %f\n", i, distance);
		//out_file << i << '\t' << newVertex.x << '\t' << newVertex.y << '\t' << newVertex.th << '\t'
		//	<< out[3] << '\t' << idx_near << '\t' << distance << std::endl;
                if (i > K && distance < 0.2){
			printf("met the goal!!\n");
		       	break;
		}
            }
            else {
                i--;
            }
        }
        else {
            i--;
        }
    }
}

point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    //TODO
    double randX = (x_max - x_min) * (double)rand() / (double)RAND_MAX;
    double randY = (y_max - y_min) * (double)rand() / (double)RAND_MAX;
    double newX = x_min + randX;
    double newY = y_min + randY;
    point newPoint;
    newPoint.x = newX;
    newPoint.y = newY;
    //printf("x, y value in randomState %f, %f\n", newPoint.x, newPoint.y);
    return newPoint;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
    //TODO
    int min_idx = -1;
    double min_dist = DBL_MAX;
    int start_idx = 0; // need check

    //temp values
    double distance, x_err, y_err, th_err;
    bool alpha_check;

    for(int i = start_idx; i < this->count; i++) {
        point x_near = this->ptrTable[i]->location;
        distance = (x_near.x - x_rand.x) * (x_near.x - x_rand.x) + (x_near.y - x_rand.y) * (x_near.y - x_rand.y);
        x_err = x_rand.x - x_near.x; 
        y_err = x_rand.y - x_near.y;
        th_err = atan2(y_err, x_err) - x_near.th; 
        
	//if the two angle btw size of atan2(y_err, x_err) and x_near.th is less than the max_alpha
	//it is true
	if(th_err <= max_alpha && th_err >= -max_alpha)alpha_check = true;
	else alpha_check = false;
        
        if ((distance < min_dist) && (distance < MaxStep*MaxStep) && alpha_check ) { ///////need check
            min_idx = i;
            min_dist = distance;
        }
    }
    return min_idx;
}

int rrtTree::nearestNeighbor(point x_rand) {
    //TODO
    //why need this?
    int min_idx;
    double min_dist = DBL_MAX;
    for(int i = 0; i < this->count; i++) {
        point tempPoint = this->ptrTable[i]->location;
        double distance = (tempPoint.x - x_rand.x) * (tempPoint.x - x_rand.x) + (tempPoint.y - x_rand.y) * (tempPoint.y - x_rand.y);
        if (distance < min_dist) {
            min_idx = i;
            min_dist = distance;
        }
    }
    return min_idx;
}

int rrtTree::newState(double *out, point x_near, point x_rand, double MaxStep) {
    //TODO
    const int n_route = 100;
    double min_distance = DBL_MAX;
    int returnvalue = 1; // decide whether to use the output, 1 : not use
    //printf("start of newStatefunction\n");
    //printf("x_rand.x = %f, x_rand.y = %f \n",x_rand.x, x_rand.y);

    //assigning a first_points.
    double first_points_x;
    double first_points_y;
    double first_err_x;
    double first_err_y;
    double first_points_th;

    //temp values to be used to saved
    double d, alpha, radius, beta, theta_alpha, theta_beta, centerX, centerY, newx, newy;
    double x_err, y_err, distance;
    int n_iter;
    bool collision;

    for (int i = 0; i < n_route; i++) {
    
        d = (car_speed / frequency) * ((double)rand()/(double)RAND_MAX); // less than car_speed / frequency
        alpha = (((double)rand()/(double)RAND_MAX) - 0.5) * 2 * max_alpha; // in [-max_alpha, max_alpha] 

        //radius = std::abs( L / tan(alpha) );
	//is this value not supposed to be absolute value?
	radius = L/tan(alpha);
        beta = d / radius;

	theta_alpha = x_near.th;
        theta_beta = x_near.th;

        centerX = x_near.x - radius * sin(x_near.th);
        centerY = x_near.y + radius * cos(x_near.th);

	n_iter = 1 + MaxStep / d;

        newx = centerX + radius * sin(x_near.th + beta);
        newy = centerY - radius * cos(x_near.th + beta);

	//calculating possible values to be saved
        first_points_x = newx;
        first_points_y = newy;
	//testing new_theta
	//first_err_x = (newx - x_near.x);
	//first_err_y = (newy - x_near.y);
	//first_points_th = atan2(first_err_y,first_err_x);
	first_points_th = x_near.th + alpha;
        
        point x1;
        point x2 = x_near;

	//printf("dead before making path?\n");
        for (int j = 0; j < n_iter; j++) {
            theta_beta = theta_beta + beta;
	    theta_alpha = theta_alpha + alpha;
            newx = centerX + radius * sin(theta_beta);
            newy = centerY - radius * cos(theta_beta);

            x1 = x2;
            x2.x = newx;
            x2.y = newy;
	    x2.th = theta_alpha;

	    //printf("dead before checking collision?\n");
            collision = isCollision(x1, x2, d, alpha);
	    if (collision){
		    //printf("collision occurred inside newState path[%d]\n", i);
		    break;
	    }
	    else {
                x_err = x_rand.x - newx;
                y_err = x_rand.y - newy;
                distance = x_err * x_err + y_err * y_err;
                if (min_distance > distance) {
                    returnvalue = 0;
                    min_distance = distance;
                    out[0] = first_points_x;
                    out[1] = first_points_y;
                    out[2] = first_points_th;
                    out[3] = alpha;
                    out[4] = d;
                }
            }
	//if(i == n_route - 1)printf("i == n_route but not dead yet\n");
        }
    }
    //printf("End of newState function\n");
    return returnvalue;
}

bool rrtTree::isCollision(point x1, point x2, double d, double alpha) {
    //TODO
    double distance_arrived = (L/this->res) * (L/this->res);
    double distance;

    double x1_i = x1.x/this->res + this->map_origin_x;
    double x1_j = x1.y/this->res + this->map_origin_y;
    double x2_i = x2.x/this->res + this->map_origin_x;
    double x2_j = x2.y/this->res + this->map_origin_y;

    //double radius = L / std::abs(tan(alpha));
    //is this value not supposed to be abs?
    double radius = L / tan(alpha);
    double beta = d / radius;

    double theta = x1.th;
    double centerX = x1_i - radius * sin(theta)/this->res;
    double centerY = x1_j + radius * cos(theta)/this->res;
    
    //number of steps calculated from the position x2 and x1.
    int max_iter = (int)(std::abs(x2_i - x1_i)+0.5) + (int)(std::abs(x2_j - x1_j) + 0.5) - 1;
    double newx, newy;

    for(int i = 0; i < max_iter; ++i){
	//newly calculated theta btw x1.theta and x2.theta
	theta = x1.th * (double)(max_iter - i)/max_iter + x2.th * (double)i/max_iter;
        newx = centerX + radius * sin(theta)/this->res;
	newy = centerY - radius * cos(theta)/this->res;
        if (this->map.at<uchar>((int)(newx+0.5), (int)(newy+0.5)) == 255) { // occupied 
           return true; // means there is a collision : cannot move
        }else if(this->map.at<uchar>((int)(newx+0.5), (int)(newy+0.5)) == 125){ // don't know
		if((this->map.at<uchar>((int)(newx+1.5), (int)(newy+0.5)) == 255) ||
		(this->map.at<uchar>((int)(newx-0.5), (int)(newy+0.5)) == 255)||
		(this->map.at<uchar>((int)(newx+0.5), (int)(newy+1.5)) == 255)||
		(this->map.at<uchar>((int)(newx+0.5), (int)(newy-0.5)) == 255))
			return true;//means regions surrounding don't know are occupied.

	}
	distance = (x2.x-newx) * (x2.x-newx) + (x2.y - newy) * (x2.y - newy);
	if(distance < distance_arrived)break;
   }
    return false;
}


std::vector<traj> rrtTree::backtracking_traj(){
    //TODO
    int idx;
    double min_distance = DBL_MAX;

    for (int i = 0; i < this->count; i++) {
        double x_err = this->x_goal.x - this->ptrTable[i]->location.x;
        double y_err = this->x_goal.y - this->ptrTable[i]->location.y;
        double distance = x_err * x_err + y_err * y_err;
        if (distance < min_distance) { // find the closest point to the goal
            idx = i;
            min_distance = distance;
        }
    }
    std::vector<traj> route;
    route.reserve(5000); // allocate memory
    traj newTraj;
    newTraj.x = x_goal.x;
    newTraj.y = x_goal.y;
    newTraj.th = this->ptrTable[idx]->location.th;
    newTraj.d = this->ptrTable[idx]->d;
    newTraj.alpha = this->ptrTable[idx]->alpha;
    route.push_back(newTraj);

    //int idx_parent = -1;
    int count = 1;

    while(idx != NULL) {
        newTraj.x = this->ptrTable[idx]->location.x;
        newTraj.y = this->ptrTable[idx]->location.y;
        newTraj.th = this->ptrTable[idx]->location.th;
        newTraj.d = this->ptrTable[idx]->d;
        newTraj.alpha = this->ptrTable[idx]->alpha;
        route.push_back(newTraj);    
        count += 1;
        idx = this->ptrTable[idx]->idx_parent;
	//if(this->ptrTable[idx]->idx_parent != NULL)	
	//	printf("idx: %d, x: %f, y: %f, th: %f, d: %f, alpha: %f, idx_parent: %d\n", idx, newTraj.x, 
	//			newTraj.y, newTraj.th, newTraj.d, newTraj.alpha, this->ptrTable[idx]->idx_parent);
	//else printf("idx_parent is NULL\n");
	//if(idx == NULL)
	//	printf("idx: %d, x: %f, y: %f, th: %f, d: %f, alpha: %f, idx_parent: %d\n", idx, newTraj.x, 
	//			newTraj.y, newTraj.th, newTraj.d, newTraj.alpha, this->ptrTable[idx]->idx_parent);
    }
    route.resize(count);
    return route;
}
