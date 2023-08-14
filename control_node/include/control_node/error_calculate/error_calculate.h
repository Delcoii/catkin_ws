#ifndef __ERROR_CALCULATE_H__
#define __ERROR_CALCULATE_H__

#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>

#define WINDOW_SIZE         5

class FollowingError {

    ros::NodeHandle nh;
    ros::Publisher error_pub;

    double car_waypoint_dist_m;
    std::vector<double> window;
    
    std_msgs::Float64 cross_track_error_m;

public:
    FollowingError();
    void GetCrossTrackError(double dist_m);
    void PutInWindow();

    double FilteredValue(double dist_m);

    void PubCrossTrackError();

};




#endif // __ERROR_CALCULATE_H__