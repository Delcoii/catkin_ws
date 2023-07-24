#ifndef __CAR_STATUS__
#define __CAR_STATUS__

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include "callback_info/CarlaEgoVehicleStatus.h"
#include "callback_info/CallbackMyCar.h"


class SubPubCarData {
private:
    
    ros::NodeHandle nh;
    ros::Subscriber odometry_sub;
    ros::Subscriber vehicle_status_sub;
    ros::Subscriber front_wheel_sub;

    ros::Publisher all_data_pub;

    void GetOdometryPose (const nav_msgs::Odometry::ConstPtr& msg);
    void GetVehicleStatus (const callback_info::CarlaEgoVehicleStatus::ConstPtr& msg);
    
    

    callback_info::CallbackMyCar car_status_msg;
    
public:
    SubPubCarData ();

    void SetFrontWheelPose (const tf::StampedTransform& tf);

    void publish_data();

    void print_status();
};

#endif // __CAR_STATUS__