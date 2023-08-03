#include "pure_pursuit/PurePursuitControl.h"


PurePursuitControl::PurePursuitControl() {

    vehicle_pos_x = 0.;
    vehicle_pos_y = 0.;
    vehicle_pos_yaw_rad = 0.;

    wypt_pos_x = 0.;
    wypt_pos_y = 0.;
    wypt_car_dist_m = 0.;
    wypt_yaw_rad = 0.;

    alpha_deg = 0.;
    theta_deg = 0.;
}


double PurePursuitControl::SetSteer(double _wypt_x, double _wypt_y) {

    UpdateWypt(_wypt_x, _wypt_y);
    GetRearPos();
    CalcWyptDist();
    CalcAlpha();
    CalcTheta();

    double steer = CutMinMax(theta_deg, (-1.)*MAX_STEERING_DEG, MAX_STEERING_DEG);
    steer = map(steer, (-1.)*MAX_STEERING_DEG, MAX_STEERING_DEG, -1., 1.);

    return steer;
}


void PurePursuitControl::GetRearPos() {
    
    double roll, pitch, yaw;
    tf::StampedTransform rr_tf;

    try {
        rr_tf_listener.lookupTransform("map", "rear_wheel_center", ros::Time(0), rr_tf);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    vehicle_pos_x = rr_tf.getOrigin().x();
    vehicle_pos_y = rr_tf.getOrigin().y(); 

    tf::Quaternion quat(
        rr_tf.getRotation().x(),
        rr_tf.getRotation().y(),
        rr_tf.getRotation().z(),
        rr_tf.getRotation().w());
    
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    vehicle_pos_yaw_rad = yaw;
}


void PurePursuitControl::UpdateWypt(double _pos_x, double _pos_y) {
    wypt_pos_x = _pos_x;
    wypt_pos_y = _pos_y;
}

void PurePursuitControl::CalcWyptDist () {
    
    wypt_car_dist_m = sqrt(
        pow((vehicle_pos_x-wypt_pos_x), 2) +
        pow((vehicle_pos_y-wypt_pos_y), 2)
    );
}

void PurePursuitControl::CalcAlpha () {
    wypt_yaw_rad = atan2(wypt_pos_y - vehicle_pos_y, wypt_pos_x - vehicle_pos_x);
    // wypt_yaw_rad = atan2(vehicle_pos_y - wypt_pos_y, vehicle_pos_x - wypt_pos_x);

    alpha_deg = rad2deg(vehicle_pos_yaw_rad) - rad2deg(wypt_yaw_rad);
    // std::cout << rad2deg(waypoint_yaw);
}

void PurePursuitControl::CalcTheta() {
    // double ld;
    // if (alpha_deg < 0.)
    //     ld = wypt_car_dist_m * -1.;
    // else
    //     ld = wypt_car_dist_m;

    theta_deg = rad2deg(
        atan((2.*WHEEL_BASE_M*sin(deg2rad(alpha_deg))) / wypt_car_dist_m)
    );

    // std::cout << 
    //     "wb : " << WHEEL_BASE_M << "\n" <<
    //     "sin term : " << sin(deg2rad(alpha_deg)) << "\n" <<
    //     (2. * WHEEL_BASE_M * sin(deg2rad(alpha_deg))) << "\n" <<
    //     "atan inside : " << (2.*WHEEL_BASE_M*sin(deg2rad(alpha_deg))) / wypt_car_dist_m << "\n" <<
    // std::endl;
}


bool PurePursuitControl::ArrivedLKDist () {
    if (wypt_car_dist_m < LOOKAHEAD_DIST)
        return true;
    else
        return false;
}

void PurePursuitControl::print_val() {
    std::cout << 
        "vehicle rear pos x : " << vehicle_pos_x << "\n" <<
        "vehicle rear pos y : " << vehicle_pos_y << "\n\n" <<
        
        "waypoint distance (m) : " << wypt_car_dist_m << "\n" << 
        "waypoint yaw(deg) : " << rad2deg(wypt_yaw_rad) << "\n" <<
        "vehicle pos yaw(deg) : " << rad2deg(vehicle_pos_yaw_rad) << "\n" <<
        "alpha(deg) : " << alpha_deg << "\n" <<
        "theta(deg) : " << theta_deg << "\n" <<
    std::endl;
}   



double PurePursuitControl::deg2rad(double deg) {
    return deg * M_PI/180.;
}
double PurePursuitControl::rad2deg(double rad) {
    return rad * 180./M_PI;
}
double PurePursuitControl::map (double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
double PurePursuitControl::CutMinMax(double x, double min, double max) {
    if (x >= max)
        return max;
    else if (x <= min)
        return min;
    else
        return x;
}