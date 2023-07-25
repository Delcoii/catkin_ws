#include "stanley_method_v2/sub_pub_class.h"


MyCarStatus::MyCarStatus () {

    car_status_sub = nh.subscribe("/callback_data", 100, &MyCarStatus::GetData, this);

    stanley_pid_pub = nh.advertise<stanley_method_v2::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 100);
}


void MyCarStatus::GetData(const stanley_method_v2::CallbackMyCar::ConstPtr& msg) {

    car_status.car_odometry = msg->car_odometry;
    car_status.front_wheel_pose = msg->front_wheel_pose;
    car_status.vehicle_status = msg->vehicle_status;

}


void MyCarStatus::InitPubMsg(const ros::Time time, const double throttle, const double steer, const double brake) {
    msg2pub.header.stamp = time;
    msg2pub.throttle = throttle;
    msg2pub.steer = steer;
    msg2pub.brake = brake;
}

void MyCarStatus::pub_data() {
    stanley_pid_pub.publish(msg2pub);
}


