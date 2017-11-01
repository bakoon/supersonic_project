#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#define PI 3.14159265358979323846
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))  
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))  
#include <cmath>

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
    newNode = new node;
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
    return newPoint;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
    //TODO
    int min_idx;
    double min_dist = DBL_MAX;

    //max_alpha
    int start_idx = 0; // need check

    for(int i = start_idx; i < this->count; i++) {
        point tempPoint = this->ptrTable[i];
        double distance = (tempPoint.x - x_rand.x) * (tempPoint.x - x_rand.x) + (tempPoint.y - x_rand.y) * (tempPoint.y - x_rand.y);
        double th_err = tempPoint.th - x_rand.th; // need check
		th_err = fmod(th_err, 2*M_PI);
		if (th_err > M_PI) {
			th_err -= 2*M_PI;
            bool alpha_check = th_err > -max_alpha;
		}
		else if (th_err < -M_PI) {
			th_err += 2*M_PI;
            bool alpha_check = th_err < max_alpha;
		}

        if (distance < min_dist && distance < MaxStep && alpha_check ) { ///////need check
            min_idx = i;
            min_dist = distance;
        }

    return min_idx;
}

int rrtTree::nearestNeighbor(point x_rand) {
    //TODO
    int min_idx;
    double min_dist = DBL_MAX;
    for(int i = 0; i < this->count; i++) {
        point tempPoint = this->ptrTable[i];
        double distance = (tempPoint.x - x_rand.x) * (tempPoint.x - x_rand.x) + (tempPoint.y - x_rand.y) * (tempPoint.y - x_rand.y);
        if (distance < min_dist) {
            min_idx = i;
            min_dist = distance;
        }
    return min_idx;


int rrtTree::newState(double *out, point x_near, point x_rand, double MaxStep) {
    //TODO
}

bool rrtTree::isCollision(point x1, point x2, double d, double alpha) {
    //TODO

    //double deltaT = 0.015;
    //double speed = 2.0;
    // d = deltaT * speed;

    double x1_i = x1.x/this->res + this->map_origin_x;
    double x1_j = x1.y/this->res + this->map_origin_y;
    double x2_i = x2.x/this->res + this->map_origin_x;
    double x2_j = x2.y/this->res + this->map_origin_y;

    double radius = L / tan(alpha);

    double theta = x1.th;
    double centerX = x1_i - radius * sin(theta);
    double centerY = x1_y - radius * cos(theta);
    
    double distance = DBL_MAX;

    while (distance > L*L) { // until when?

        double beta = d / radius;

        double newx = centerX + radius * sin(theta + beta);
        double newy = centery - radius * con(theta + beta);

        theta = theta + beta;
        distance = (x2.x - newx) * (x2.x - newx) + (x2.y - newy) * (x2.y - newy);
        if (map_margin.at<uchar>((int)(newx+0.5), (int)(newy+0.5)) == 0) { // occupied 
        //if (map_margin.at<uchar>((int)(newx+0.5), (int)(newy+0.5)) == 0 || map_margin.at<uchar>((int)newx, (int)newy) == 125 ) { // occupied or unknown
        // ***add*** when 125, check around
            return true;
        }
    }
    return false;
}


std::vector<traj> rrtTree::backtracking_traj(){
    //TODO
}
