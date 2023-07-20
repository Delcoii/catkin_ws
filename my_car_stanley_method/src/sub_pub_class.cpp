#include "my_car_stanley_method/sub_pub_class.h"


MyCarStatus::MyCarStatus () {

    car_status_sub = nh.subscribe("/carla/ego_vehicle/vehicle_status", 100, &MyCarStatus::GetVelocity, this);

    front_wheel_pose_sub = nh.subscribe("/front_wheel_pub", 100, &MyCarStatus::GetFLFRCenterPose, this);

    stanley_pid_pub = nh.advertise<my_car_stanley_method::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 100);

}


void MyCarStatus::GetVelocity(const my_car_stanley_method::CarlaEgoVehicleStatus::ConstPtr& msg) {
    // std::cout << "get!!" << std::endl;
    this->velocity_ms = msg->velocity;
}

void MyCarStatus::GetFLFRCenterPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    front_wheel_pos_x = msg->pose.position.x;    
    front_wheel_pos_y = msg->pose.position.y;    
    front_wheel_pos_z = msg->pose.position.z;    

}


void MyCarStatus::InitPubMsg(const ros::Time time, const double throttle, const double steer) {
    msg2pub.header.stamp = time;
    msg2pub.throttle = throttle;
    msg2pub.steer = steer;
}

void MyCarStatus::pub_data() {
    stanley_pid_pub.publish(msg2pub);
}


void MyCarStatus::print_val() {
    std::cout <<
        "subscribed data : \n" << 
        "velocity(m/s) : " << velocity_ms << "\n" <<
        "front wheel :\n" <<
        "    x\t: " << front_wheel_pos_x << "\n" << 
        "    y\t: " << front_wheel_pos_y << "\n" <<
        "    z\t: " << front_wheel_pos_z << "\n" <<
    std::endl;

    std::cout <<
        "calculated data :\n" <<
    
    std::endl;

    std::cout << 
        "publishing data : \n" << 
        "throttle : " << msg2pub.throttle << "\n" << 
        "========" <<
    std::endl;
}