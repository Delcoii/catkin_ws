#ifndef __CONTROL_NODE_H__
#define __CONTROL_NODE_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "pure_pursuit/CarlaEgoVehicleStatus.h"
#include "pure_pursuit/CarlaEgoVehicleControl.h"

class ControlMsgs {

    ros::NodeHandle nh;
    
    
    ros::Subscriber vehicle_status_sub;
    ros::Publisher control_pub;

    pure_pursuit::CarlaEgoVehicleControl ctrl_msg;

public:

    ControlMsgs ();
    void GetCarStatus(const pure_pursuit::CarlaEgoVehicleStatus::ConstPtr& msg);

    pure_pursuit::CarlaEgoVehicleStatus vehicle_status;
    void PubMsg(double throttle, double steer, double brake);
};






#endif // __CONTROL_NODE_H__