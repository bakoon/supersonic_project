#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#define PI 3.14159265358979323846
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))  
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))  
#include <cmath>

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
    //printf("generateRRT start\n");
    int count = 0;
    const int max_route = 20000;
    //for (int i = 0; i < max_route; ++i) {
    while (count < max_route) {
        printf("point # %d\n", count);
        point randVertex = randomState(x_max, x_min, y_max, y_min);
        int idx_near = nearestNeighbor(randVertex, MaxStep);
        if (idx_near != -1) {
            //printf("nearestNeighbor ok\n");
            //printf("idx_near = %d\n", idx_near);
            double out[5];
            int getNewState = this->newState(out, ptrTable[idx_near]->location, randVertex, MaxStep);
            //printf("getNewState %d\n", getNewState);
            if (getNewState == 0) {
                count++;
                point newVertex;
                newVertex.x = out[0];
                newVertex.y = out[1];
                newVertex.th = out[2];
                double alpha = out[3];
                double d = out[4];
                
                //printf("newV x, y, th %.3f, %.3f, %.3f\n", out[0], out[1], out[2]);
                addVertex(newVertex, randVertex, idx_near, alpha, d);
                //printf("before distance\n");
                double distance = (newVertex.x - this->x_goal.x)*(newVertex.x - this->x_goal.x) + (newVertex.y - this->x_goal.y)*(newVertex.y - this->x_goal.y);
                //printf("distance %.3f\n", distance);
                //double distance = std::sqrt((out[0] - this->x_goal.x)*(out[0] - this->x_goal.x) + (out[1] - this->x_goal.y)*(out[1] - this->x_goal.y));
                if (count > K && distance < 0.2*0.2) break;
            }
        }
    //printf("RRT node # %d\n", i);
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
    //printf("randomState x, y : %p, %.3f, %.3f\n", &newPoint, newX, newY);
    return newPoint;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
    //TODO
    int min_idx = -1;
    double min_dist = DBL_MAX;
    point temp = x_rand;
    double x = x_rand.x;
    double y = x_rand.y;
    //printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", x, y, x_rand.x, x_rand.y, temp.x, temp.y);

    //max_alpha
    int start_idx = 0; // need check

    for(int i = start_idx; i < this->count; i++) {
        point x_near = this->ptrTable[i]->location;
        //printf("%d, %d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", x_near.x, x_near.y, x_rand.x, x_rand.y, x, y, temp.x, temp.y);
        double distance = (x_near.x - x_rand.x) * (x_near.x - x_rand.x) + (x_near.y - x_rand.y) * (x_near.y - x_rand.y);
        double x_err = x_rand.x - x_near.x; 
        double y_err = x_rand.y - x_near.y;
        double th_err = atan2(y_err, x_err);  
        bool alpha_check;
		if (th_err <= -M_PI) {
            alpha_check = th_err > -max_alpha;
		}
		else if (th_err >= M_PI) {
            alpha_check = th_err < max_alpha;
		}
        //printf("min_dist = %.3f, MaxStep = %.3f, th_err : %.3f\n", min_dist, MaxStep*MaxStep, th_err);

        if (distance < min_dist && distance < MaxStep*MaxStep && alpha_check ) { ///////need check
            min_idx = i;
            min_dist = distance;
        }
    }
    //printf("nearest neighbor : %d\n", min_idx);
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
    //printf("newstate\n");
   
    const int max_try = 5;
    point newPoint;
    point nearX;
    nearX.x = x_near.x;
    nearX.y = x_near.y;
    nearX.th = x_near.th;
    point randX;
    randX.x = x_rand.x;
    randX.y = x_rand.y;
    randX.th = x_rand.th;
    //printf("xnear %.3f, %.3f, xrand %.3f, %.3f\n", nearX.x, nearX.y, randX.x, randX.y);

    for (int i = 0; i < max_try; i++) {
        double alpha = (((double)rand()/(double)RAND_MAX) - 0.5) * 2 * max_alpha; // in [-max_alpha, max_alpha] 

        double radius = std::abs(L / tan(alpha));
        double theta = x_near.th;

        double centerX = nearX.x - radius * sin(theta);
        double centerY = nearX.y - radius * cos(theta);

        double xnear_from_center_x = centerX - nearX.x;
        double xnear_from_center_y = centerY - nearX.y;
        double xrand_from_center_x = centerX - randX.x;
        double xrand_from_center_y = centerY - randX.y;

        double x_err = xnear_from_center_x - xrand_from_center_x;
        double y_err = xnear_from_center_y - xrand_from_center_y;

        double angle_near_rand = atan2(y_err, x_err);
        double arc_length = radius*angle_near_rand;
        double beta;

        if (arc_length > MaxStep) {
            beta = 0.1 * angle_near_rand * MaxStep / (2*arc_length);
            newPoint.x = centerX + radius * sin(theta + beta);
            newPoint.y = centerY - radius * cos(theta + beta);
        }
        else {
            beta = 0.1 * angle_near_rand;
            newPoint.x = randX.x;
            newPoint.y = randX.y;
        }
        double d = beta * radius;
        
        //printf("%d bef collisioncheck\n", i);
        bool collisionCheck = isCollision(nearX, newPoint, d, alpha);
        //printf("%daft collisioncheck\n", i);

        if (!collisionCheck) {
        
            out[0] = newPoint.x;
            out[1] = newPoint.y;
            out[2] = atan2(centerX-out[0], out[1]-centerY);
            out[3] = alpha;
            out[4] = d;
            return 0;
        }
    }
    return 1;
}

bool rrtTree::isCollision(point x1, point x2, double d, double alpha) {
    //TODO

    //double deltaT = 0.015;
    //double speed = 2.0;
    // d = deltaT * speed;
    const double map_max = 800;

    //printf("x1 %.3f, %.3f, x2 %.3f, %.3f\n", x1.x, x1.y, x2.x, x2.y);

    double distance_arrived = (L/this->res) * (L/this->res);

    double x1_i = x1.x/this->res + this->map_origin_x;
    double x1_j = x1.y/this->res + this->map_origin_y;
    double x2_i = x2.x/this->res + this->map_origin_x;
    double x2_j = x2.y/this->res + this->map_origin_y;

    double radius = L / std::abs(tan(alpha));
    //printf("radius : %.3f\n", radius);
    double beta = d / radius;

    double theta = x1.th;
    double centerX = x1_i - radius * sin(theta) / this->res;
    double centerY = x1_j + radius * cos(theta) / this->res;
    
    int max_iter = 10;

    double newx = x1_i;
    double newy = x1_j;

    for (int i = 0; i < max_iter; i++) {

        theta = theta + beta;
        newx = MAX(0, MIN(map_max, centerX + radius * sin(theta)));
        newy = MAX(0, MIN(map_max, centerY - radius * cos(theta)));
        if (newx*newy == 0 || newx == map_max || newy == map_max) {
            return false;
        }

        //printf("collisioncheck %d, x, y : %.3f, %.3f\n", i, newx, newy);

        int collisionInfo = this->map.at<uchar>((int)(newx+0.5), (int)(newy+0.5));
        //printf("collision: %d\n", collisionInfo);

        if (collisionInfo == 0 || collisionInfo == 125) { // occupied 
        //if (this->map.at<uchar>((int)(newx+0.5), (int)(newy+0.5)) == 0 || this->map.at<uchar>((int)newx, (int)newy) == 125 ) { // occupied or unknown
        // ***need add*** when 125, check around??
            return true; // means there is a collision : cannot move
        }

    }
    return false;
}


std::vector<traj> rrtTree::backtracking_traj(){
    //TODO
    int idx = -1;
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
    traj newTraj;
    newTraj.x = x_goal.x;
    newTraj.y = x_goal.y;
    newTraj.th = this->ptrTable[idx]->location.th;
    newTraj.d = this->ptrTable[idx]->d;
    newTraj.alpha = 0;//this->ptrTable[idx]->alpha;
    route.push_back(newTraj);

    while(idx != NULL) {
        //printf("backtracking idx %d\n", idx);
        newTraj.x = this->ptrTable[idx]->location.x;
        newTraj.y = this->ptrTable[idx]->location.y;
        newTraj.th = this->ptrTable[idx]->location.th;
        newTraj.d = this->ptrTable[idx]->d;
        newTraj.alpha = this->ptrTable[idx]->alpha;
        route.push_back(newTraj);    
        idx = this->ptrTable[idx]->idx_parent;
    }
    //delete[] this->ptrTable;
    //printf("backtracking return\n");
    return route;
}
