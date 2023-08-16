#ifndef __PURE_PURSUIT_CONTROL_H__
#define __PURE_PURSUIT_CONTROL_H__

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

#include "control_node/math_functions/math_functions.h"

#define LOOKAHEAD_DIST_MIN  5.
#define LOOKAHEAD_GRADIENT  2.
#define SEARCH_IDX_RANGE    100

#define MAX_STEERING_DEG    34.9999
#define WHEEL_BASE_M        2.886

class PurePursuitControl {

    double vehicle_pos_x;   // rear pos
    double vehicle_pos_y;   // rear pos
    double vehicle_pos_yaw_rad;

    double lookahead_dist_m;

    double wypt_pos_x;
    double wypt_pos_y;
    double wypt_car_dist_m;
    double wypt_yaw_rad;

    double alpha_deg;
    double theta_deg;

    double steering_val;
    
    std::vector<std::vector<double>> waypoints;
    int target_wypt_idx;

public:

    PurePursuitControl();

    double SetSteer(geometry_msgs::PoseStamped _rr_pose, double vel_ms);

    void GetAllWaypoints(std::vector<std::vector<double>> wypts);

    void FindTargetPoint(geometry_msgs::PoseStamped _rr_pose, double vel_ms);
    void GetRearPos(geometry_msgs::PoseStamped _rr_pose);
    void CalcWyptDist();
    void CalcAlpha();
    void CalcTheta();

    // bool ArrivedLKDist();

    double steer_val();
    int target_idx();

    void PrintValue();
};



#endif // __PURE_PURSUIT_CONTROL_H__