#include "pure_pursuit/ControlMsgs/ControlMsgs.h"


ControlMsgs::ControlMsgs () {

    vehicle_status_sub = nh.subscribe ("/carla/ego_vehicle/vehicle_status", 100, &ControlMsgs::GetCarStatus, this);

    control_pub = nh.advertise<pure_pursuit::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 100);


    // car_odo_sub = nh.subscribe ("/carla/ego_vehicle/odometry", 100, &ControlMsgs::GetCarOdo, this);

}


// void ControlMsgs::GetCarOdo(const nav_msgs::Odometry::ConstPtr& msg) {
//     car_odometry.pose = msg->pose;
// }
 
void ControlMsgs::GetCarStatus(const pure_pursuit::CarlaEgoVehicleStatus::ConstPtr& msg) {
    
    vehicle_status.velocity = msg->velocity;
    vehicle_status.acceleration = msg->acceleration;
    vehicle_status.orientation = msg->orientation;
    vehicle_status.control = msg->control;
}

void ControlMsgs::PubMsg(double throttle, double steer, double brake) {
    ctrl_msg.throttle = throttle;
    ctrl_msg.steer = steer;
    ctrl_msg.brake = brake;

    control_pub.publish(ctrl_msg);
}