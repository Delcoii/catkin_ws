#ifndef __SUB_PUB_CLASS_H__
#define __SUB_PUB_CLASS_H__

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "my_car_stanley_method/CarlaEgoVehicleControl.h"
#include "my_car_stanley_method/CarlaEgoVehicleStatus.h"


class MyCarStatus {


    ros::NodeHandle nh;

    ros::Subscriber car_status_sub;
    ros::Subscriber car_odometry_sub;
    ros::Subscriber front_wheel_pose_sub;

    my_car_stanley_method::CarlaEgoVehicleControl msg2pub;

    ros::Publisher stanley_pid_pub;
    ros::Publisher front_wheel_pose_pub;
    

public:

    // subscribed data from /ego_vehicle/vehicle_status 
    double velocity_ms;
    geometry_msgs::Pose front_wheel_center;
    nav_msgs::Odometry car_odometry;


    MyCarStatus();

    void GetVelocity(const my_car_stanley_method::CarlaEgoVehicleStatus::ConstPtr& msg);
    void GetOdometryPose(const nav_msgs::Odometry::ConstPtr& msg);
    void GetFLFRCenterPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void InitPubMsg(const ros::Time time, const double throttle, const double steer);
    void pub_data();
    void print_val();
    
};


#endif // __SUB_PUB_CLASS_H__