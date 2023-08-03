#ifndef __SUB_PUB_H__
#define __SUB_PUB_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "pure_pursuit/CarlaEgoVehicleStatus.h"
#include "pure_pursuit/CarlaEgoVehicleControl.h"

class ControlMsgs {

    ros::NodeHandle nh;
    
    ros::Subscriber car_odo_sub;
    ros::Subscriber vehicle_status_sub;
    
    ros::Publisher control_pub;


    void GetCarOdo(const nav_msgs::Odometry::ConstPtr& msg);
    void GetCarStatus(const pure_pursuit::CarlaEgoVehicleStatus::ConstPtr& msg);

public:

    ControlMsgs ();

    nav_msgs::Odometry car_odometry;
    pure_pursuit::CarlaEgoVehicleStatus vehicle_status;
    
};


#endif // __SUB_PUB_H__