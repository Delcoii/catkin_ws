#ifndef __ERROR_CALCULATE_H__
#define __ERROR_CALCULATE_H__

#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>

#include "control_node/ErrorMsgs.h"

#define WINDOW_SIZE         50

class FollowingError {

    ros::NodeHandle nh;
    ros::Publisher error_pub;

    double car_waypoint_dist_m;
    
    std::vector<double> window;     // for moving average filter

    double average;                 // for normal average filter
    double sample_count;
    
    control_node::ErrorMsgs errors;

public:
    FollowingError();
    void GetCrossTrackError(double dist_m);
    void PutInWindow();     // for cross track error filtering

    // CTE : cross track error
    double FilteringCTE(double dist_m);
    double cte_err_avg();

    void GetSpeed(double target_speed, double now_speed);

    void PubError();

};




#endif // __ERROR_CALCULATE_H__