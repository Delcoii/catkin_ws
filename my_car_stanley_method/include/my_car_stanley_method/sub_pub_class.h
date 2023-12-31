#ifndef __SUB_PUB_CLASS_H__
#define __SUB_PUB_CLASS_H__

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "my_car_stanley_method/CarlaEgoVehicleControl.h"
#include "my_car_stanley_method/CarlaEgoVehicleStatus.h"


class MyCarStatus {


    ros::NodeHandle nh;
    ros::Publisher stanley_pid_pub;
    ros::Subscriber car_status_sub;
    ros::Subscriber front_wheel_pose_sub;
    ros::Subscriber car_odometry_sub;


    my_car_stanley_method::CarlaEgoVehicleControl msg2pub;


public:

    // subscribed data from /ego_vehicle/vehicle_status 
    double velocity_ms;
    
    double front_wheel_pos_x;
    double front_wheel_pos_y;
    double front_wheel_pos_z;

    nav_msgs::Odometry car_odometry;

    

    MyCarStatus();

    void GetVelocity(const my_car_stanley_method::CarlaEgoVehicleStatus::ConstPtr& msg);
    void GetFLFRCenterPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void GetOdometryPose(const nav_msgs::Odometry::ConstPtr& msg);

    void InitPubMsg(const ros::Time time, const double throttle, const double steer);
    void pub_data();
    void print_val();
    
};


#endif // __SUB_PUB_CLASS_H__