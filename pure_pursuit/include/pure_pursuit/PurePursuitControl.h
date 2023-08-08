#ifndef __PURE_PURSUIT_CONTROL_H__
#define __PURE_PURSUIT_CONTROL_H__

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>


#define LOOKAHEAD_DIST      30.
#define WHEEL_BASE_M        2.886
#define MAX_STEERING_DEG    34.9999

class PurePursuitControl {

    tf::TransformListener rr_tf_listener;

    double vehicle_pos_x;   // rear pos
    double vehicle_pos_y;   // rear pos
    double vehicle_pos_yaw_rad;

    double wypt_pos_x;
    double wypt_pos_y;
    double wypt_car_dist_m;
    double wypt_yaw_rad;

    double alpha_deg;
    double theta_deg;

public:

    PurePursuitControl();

    double SetSteer(double _wypt_x, double _wypt_y);

    void GetRearPos();
    void CalcWyptDist();
    void UpdateWypt(double _pos_x, double _pos_y);
    void CalcAlpha();
    void CalcTheta();

    bool ArrivedLKDist();

    void print_val();

    double deg2rad(double deg);
    double rad2deg(double rad);
    double map(double x, double in_min, double in_max, double out_min, double out_max);
    double CutMinMax(double x, double min, double max);
};



#endif // __PURE_PURSUIT_CONTROL_H__