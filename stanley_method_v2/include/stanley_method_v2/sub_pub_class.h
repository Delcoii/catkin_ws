#ifndef __SUB_PUB_CLASS_H__
#define __SUB_PUB_CLASS_H__


#include <ros/ros.h>

#include "stanley_method_v2/CarlaEgoVehicleControl.h"
#include "stanley_method_v2/CallbackMyCar.h"


class MyCarStatus {

    ros::NodeHandle nh;

    ros::Subscriber car_status_sub;

    stanley_method_v2::CarlaEgoVehicleControl msg2pub;
    ros::Publisher stanley_pid_pub;
    
public:

    stanley_method_v2::CallbackMyCar car_status;

    MyCarStatus();

    void GetData(const stanley_method_v2::CallbackMyCar::ConstPtr& msg);

    void InitPubMsg(const ros::Time time, const double throttle, const double steer);
    void pub_data();
    
};


#endif // __SUB_PUB_CLASS_H__