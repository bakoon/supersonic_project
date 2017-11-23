#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#define PI 3.14159265358979323846
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))  
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))  
#include <cmath>

double max_alpha = 0.15;
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
    //cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    //cv::imshow("Mapping", imgResult(imgROI));
    cv::imshow("Mapping", imgResult);
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
    //cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    //cv::imshow("Mapping", imgResult(imgROI));
    cv::imshow("Mapping", imgResult);
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
    const int goal_bias = 5;
    bool bool_goal_bias = false;
    //for (int i = 0; i < max_route; ++i) {
    while (count < max_route) {
        point randVertex;
        if (bool_goal_bias && count % goal_bias == 0) {
            //printf("bias\n");
            randVertex.x = x_goal.x;
            randVertex.y = x_goal.y;
            //randVertex.th = x_goal.th;
            bool_goal_bias = false;
        }
        else {
            randVertex = randomState(x_max, x_min, y_max, y_min);
        }
        int idx_near = nearestNeighbor(randVertex, MaxStep);

        //printf("idx_near = %d\n", idx_near);
        if (idx_near != -1) {
            //printf("nearestNeighbor ok\n");
            double out[5];
            int getNewState = this->newState(out, ptrTable[idx_near]->location, randVertex, MaxStep);
            //printf("getNewState %d\n", getNewState);
            if (getNewState == 0) {
                count++;
                bool_goal_bias = true;
                printf("RRT node # %d, %.3f, %.3f, %.3f\n", count, out[0], out[1], out[2]);
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
                if (count > K && distance < 0.2*0.2) {
                    break;
                    //printf("early stop\n");
                }
            }
        }
    }
    //printf("generateRRT fin\n");
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
    double min_ang = DBL_MAX;
    point temp = x_rand;
    double x = x_rand.x;
    double y = x_rand.y;

    //max_alpha
    int start_idx = 0; // need check

    for(int i = start_idx; i < this->count; i++) {
        point x_near = this->ptrTable[i]->location;
        //printf("%d, %d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", x_near.x, x_near.y, x_rand.x, x_rand.y, x, y, temp.x, temp.y);
        double distance = (x_near.x - x_rand.x) * (x_near.x - x_rand.x) + (x_near.y - x_rand.y) * (x_near.y - x_rand.y);
        double x_err = x_rand.x - x_near.x; 
        double y_err = x_rand.y - x_near.y;
        //------------------------------I have changed this for test---------//
        double th_err = x_near.th - atan2(y_err, x_err);  
        //printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", x, y, x_near.x, x_near.y, x_near.th, th_err);
        bool alpha_check;
		if (th_err <= -M_PI) {
            th_err += 2*M_PI;
            alpha_check = th_err > -max_alpha;
		}
		else if (th_err >= M_PI) {
            th_err -= 2*M_PI;
            alpha_check = th_err < max_alpha;
		}

        if(th_err <= max_alpha && th_err >= -max_alpha)alpha_check = true;
        else alpha_check = false;
        /*
        */
        //printf("min_dist = %.3f, MaxStep = %.3f, th_err : %.3f, %d\n", MIN(100, min_dist), MaxStep*MaxStep, th_err, alpha_check);

        //if (distance < min_dist && distance < MaxStep*MaxStep && alpha_check ) { ///////need check
        //testing whether distance < MaxStep effect the result
        //if (distance < min_dist  && alpha_check ) { ///////need check
        //if (std::abs(th_err) < min_ang && distance > L * L && distance < MaxStep*MaxStep && alpha_check ) { ///////need check
        if (distance < 0.2 * 0.2) {
            //printf("exists near point already\n");
            min_idx = -1;
            break;
            //None
        }
        else if (std::abs(th_err) < min_ang && distance < MaxStep*MaxStep && alpha_check ) { ///////need check
            min_idx = i;
            //printf("break idx %d\n", i);
            //break;
            min_dist = distance;
            min_ang = std::abs(th_err);
            /*
            if (rand() % 10 == 0) {
                break; // probabilistic early stop
            }
            */
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

    const int max_try = 100;//5000;
    double min_distance = DBL_MAX;
    double new_d, new_alpha, new_beta;
    point newPoint;
    point nearX = x_near;
    point randX = x_rand;
    //printf("xnear %.3f, %.3f, xrand %.3f, %.3f\n", nearX.x, nearX.y, randX.x, randX.y);

    for (int i = 0; i < max_try; i++) {
        double alpha = (((double)rand()/(double)RAND_MAX) - 0.5) * 2 * max_alpha; // in [-max_alpha, max_alpha] 
        //double random_number = ((double)rand()/(double)RAND_MAX) * 2;
        //double d = MIN(random_number * MaxStep, MaxStep);
        //double d = MaxStep;
        const int stepSize = 10;
        double d = (double)(rand()%stepSize + 1) / (double)stepSize * MaxStep;
        //printf("alpha %.3f, d %.3f\n", alpha, d);
        //int max_iter = (int)(MaxStep / d);

        double radius = L / tan(alpha);
        double theta = x_near.th;

        double centerX = nearX.x - radius * sin(theta);
        double centerY = nearX.y + radius * cos(theta);

        double beta = d / radius;
        //beta = beta * max_iter;
        
        double newX = centerX + radius * sin(theta + beta);
        double newY = centerY - radius * cos(theta + beta);

        //double x_err = newX - x_rand.x;
        //double y_err = newY - x_rand.y;
        double x_err = newPoint.x - x_rand.x;
        double y_err = newPoint.y - x_rand.y;

        double distance = x_err * x_err + y_err * y_err;
        //if (distance < min_distance) {
        if (distance < min_distance && distance > L * L) {
            min_distance = distance;
            newPoint.x = newX;
            newPoint.y = newY;
            new_d = d;
            new_alpha = alpha;
            new_beta = beta;
            //if ( distance < 2 * L * L ) {
                //printf("break\n");
            //    break;
            //}
        }
    }
        
    //printf("%d bef collisioncheck\n", i);
    bool collisionCheck = isCollision(nearX, newPoint, new_d, new_alpha);
    //printf("%daft collisioncheck\n", i);

    if (!collisionCheck) {
        //printf("accept\n");
        out[0] = newPoint.x;
        out[1] = newPoint.y;
        //out[2] = atan2(-(newPoint.y - randX.y), (newPoint.x - randX.x));
        double out_th = x_near.th + new_beta;
        out_th = fmod(out_th, 2*M_PI);
        //printf("th test %.3f ", out_th);
        if (out_th > M_PI) {
            out_th -= 2*M_PI;
        }
        else if (out_th < -M_PI) {
            out_th += 2*M_PI;
        }
        //printf("new theta %.3f\n", out_th);
        out[2] = out_th; 
        out[3] = new_alpha;
        out[4] = new_d;
        return 0;
    }
    //printf("collision\n");
    return 1;
}

bool rrtTree::isCollision(point x1, point x2, double d, double alpha) {
    //TODO
    //printf("\n\n\n\nx2 %.3f, %.3f\n", x2.x, x2.y);
    bool check_enterance = false;

    int map_row_size = this->map.rows; // size info;
    int map_col_size = this->map.cols;
    //printf("map %d, %d\n", map_row_size, map_col_size);

    double x1_i = x1.x/this->res + this->map_origin_x;
    double x1_j = x1.y/this->res + this->map_origin_y;
    double x2_i = x2.x/this->res + this->map_origin_x;
    double x2_j = x2.y/this->res + this->map_origin_y;

    double radius = L / tan(alpha);
    //printf("alpha %.3f, radius : %.3f\n", alpha, radius);
    double beta = d / radius;
    //printf("beta %.3f\n", beta);

    double theta = x1.th;
    double centerX = x1_i - radius * sin(theta) / this->res;
    double centerY = x1_j + radius * cos(theta) / this->res;
    //printf("center : %.3f, %.3f\n", centerX, centerY);
    
    double angle_x1 = atan2( (x1_j - centerY) , (x1_i - centerX) );
    double angle_x2 = atan2( (x2_j - centerY) , (x2_i - centerX) );
    
    double angle_x1_x2 = angle_x1 - angle_x2;
    angle_x1_x2 = fmod(angle_x1_x2, 2*M_PI);
    if (angle_x1_x2 > M_PI) {
        angle_x1_x2 -= 2*M_PI;
    }
    else if (angle_x1_x2 < -M_PI) {
        angle_x1_x2 += 2*M_PI;
    }

    double arc_length = std::abs(angle_x1_x2 * radius / this->res);
    //printf("arc_length %.3f\n", arc_length);
    int max_iter = std::ceil(arc_length);
    //printf("max_iter : %d\n", max_iter);

    double newx = x1_i;
    double newy = x1_j; // start point
    //printf("\n\n\nmax_iter %d, x1 %.1f, %.1f, %.1f, %.1f, x2 %.1f, %.1f, %.1f, %.1f\n", max_iter, x1.x, x1.y, x1_i, x1_j, x2.x, x2.y, x2_i, x2_j);
    //printf("col size : %d, row size : %d\n", map_col_size, map_row_size);


    for (int i = 1; i <= max_iter; i++) {
        check_enterance = true;

        //printf("be radius %.3f, theta %.3f\n", radius, theta);
        theta = theta + beta / (double)max_iter;
        //printf("af radius %.3f, theta %.3f\n", radius, theta);

        newx = MAX(0, MIN(map_col_size-1, centerX + radius * sin(theta) / this->res));
        newy = MAX(0, MIN(map_row_size-1, centerY - radius * cos(theta) / this->res));
        //printf("%.3f, %.3f\n", map_row_size-1, centerY - radius * cos(theta) / this->res);
        if (newx*newy == 0 || newx == map_col_size || newy == map_row_size) {
            return true;
        }

        //printf("collisioncheck %d, x, y : %.3f, %.3f\n", i, newx, newy);
        int collisionInfo = this->map.at<uchar>((int)(newx+0.5), (int)(newy+0.5));
        //printf("%d, newx %d, newy %d, collision: %d\n", i, (int)newx, (int)newy, collisionInfo);

        if (collisionInfo == 0) {// || (collisionInfo == 125)) { // occupied 
            //printf("collision detected\n");
            return true; // means there is a collision : cannot move
        }
        //printf("no collision\n");

    }
    //printf("collision\n");
    if (!check_enterance) {
        return true;
    }
    else {
        return false;
    }



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
    
    idx = this->ptrTable[idx]->idx_parent; // ignore the final point : near goal

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
    //printf("backtracking done\n");
    return route;
}
