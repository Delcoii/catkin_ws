#ifndef __SUB_PUB_CLASS_H__
#define __SUB_PUB_CLASS_H__


#include <ros/ros.h>

#include "only_going_forward/CarlaEgoVehicleControl.h"
#include "only_going_forward/CallbackMyCar.h"


class MyCarStatus {

    ros::NodeHandle nh;

    ros::Subscriber car_status_sub;

    only_going_forward::CarlaEgoVehicleControl msg2pub;
    ros::Publisher stanley_pid_pub;
    
public:

    only_going_forward::CallbackMyCar car_status;

    MyCarStatus();

    void GetData(const only_going_forward::CallbackMyCar::ConstPtr& msg);

    void InitPubMsg(const ros::Time time, const double throttle, const double steer, const double brake);
    void pub_data();
    
};


#endif // __SUB_PUB_CLASS_H__