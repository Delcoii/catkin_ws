#include "my_car_stanley_method/sub_pub_class.h"


MyCarStatus::MyCarStatus () {

    car_status_sub = nh.subscribe("/carla/ego_vehicle/vehicle_status", 100, &MyCarStatus::GetVelocity, this);

    front_wheel_pose_sub = nh.subscribe("/front_wheel_pub", 100, &MyCarStatus::GetFLFRCenterPose, this);
    
    car_odometry_sub = nh.subscribe("/carla/ego_vehicle/odometry", 100, &MyCarStatus::GetOdometryPose, this);

    stanley_pid_pub = nh.advertise<my_car_stanley_method::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 100);


}


void MyCarStatus::GetVelocity(const my_car_stanley_method::CarlaEgoVehicleStatus::ConstPtr& msg) {
    // std::cout << "get!!" << std::endl;
    this->velocity_ms = msg->velocity;
}

void MyCarStatus::GetFLFRCenterPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //std::cout << "get!!" << std::endl;
    front_wheel_center.position.x = msg->pose.position.x;    
    front_wheel_center.position.y = msg->pose.position.y;    
    front_wheel_center.position.z = msg->pose.position.z;
    front_wheel_center.orientation.x = msg->pose.orientation.x;
    front_wheel_center.orientation.y = msg->pose.orientation.y;
    front_wheel_center.orientation.z = msg->pose.orientation.z;
    front_wheel_center.orientation.w = msg->pose.orientation.w;
}


void MyCarStatus::GetOdometryPose(const nav_msgs::Odometry::ConstPtr& msg) {
    car_odometry.pose.pose.position.x = msg->pose.pose.position.x;
    car_odometry.pose.pose.position.y = msg->pose.pose.position.y;
    car_odometry.pose.pose.position.z = msg->pose.pose.position.z;

    car_odometry.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    car_odometry.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    car_odometry.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    car_odometry.pose.pose.orientation.w = msg->pose.pose.orientation.w;
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
        "front wheel's center :\n" <<
        "position : \n" << 
        "    x : " << front_wheel_center.position.x << "\n" << 
        "    y : " << front_wheel_center.position.y << "\n" <<
        "    z : " << front_wheel_center.position.z << "\n" <<
    std::endl;



    std::cout << 
        "publishing data : \n" << 
        "throttle\t: " << msg2pub.throttle << "\n" << 
        "steeering\t: " << msg2pub.steer << "\n" <<
        "========" <<
    std::endl;
}