#include "callback_info/car_status.h"

SubPubCarData::SubPubCarData() {
    
    odometry_sub = nh.subscribe("/carla/ego_vehicle/odometry", 100, &SubPubCarData::GetOdometryPose, this);

    vehicle_status_sub = nh.subscribe("/carla/ego_vehicle/vehicle_status", 100, &SubPubCarData::GetVehicleStatus, this);

    all_data_pub = nh.advertise<callback_info::CallbackMyCar>("/callback_data", 100);
}


void SubPubCarData::GetOdometryPose (const nav_msgs::Odometry::ConstPtr& msg) {
    car_status_msg.car_odometry.pose.pose.position.x = msg->pose.pose.position.x;
    car_status_msg.car_odometry.pose.pose.position.y = msg->pose.pose.position.y;
    car_status_msg.car_odometry.pose.pose.position.z = msg->pose.pose.position.z;
    car_status_msg.car_odometry.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    car_status_msg.car_odometry.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    car_status_msg.car_odometry.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    car_status_msg.car_odometry.pose.pose.orientation.w = msg->pose.pose.orientation.w;
}

void SubPubCarData::GetVehicleStatus(const callback_info::CarlaEgoVehicleStatus::ConstPtr& msg) {
    car_status_msg.vehicle_status.velocity = msg->velocity;
    car_status_msg.vehicle_status.acceleration = msg->acceleration;
    car_status_msg.vehicle_status.orientation = msg->orientation;
    car_status_msg.vehicle_status.control = msg->control;
}

void SubPubCarData::SetFrontWheelPose (const tf::StampedTransform& tf) {
    car_status_msg.front_wheel_pose.position.x = tf.getOrigin().x();
    car_status_msg.front_wheel_pose.position.y = tf.getOrigin().y();
    car_status_msg.front_wheel_pose.position.z = tf.getOrigin().z();
    car_status_msg.front_wheel_pose.orientation.x = tf.getRotation().x();
    car_status_msg.front_wheel_pose.orientation.y = tf.getRotation().y();
    car_status_msg.front_wheel_pose.orientation.z = tf.getRotation().z();
    car_status_msg.front_wheel_pose.orientation.w = tf.getRotation().w();
}


void SubPubCarData::publish_data() {
    car_status_msg.header.stamp = ros::Time::now();
    all_data_pub.publish(car_status_msg);
}

void SubPubCarData::print_status() {
    

    std::cout << 
        "vehicle status\n" <<
        "  velocity(m/s) \t: " << car_status_msg.vehicle_status.velocity << "\n\n" << 
        // "  acceleration (m/s^2) " << "\n" <<
        // "    linear x\t\t: " << car_status_msg.vehicle_status.acceleration.linear.x << "\n" <<
        // "    linear y\t\t: " << car_status_msg.vehicle_status.acceleration.linear.y << "\n" << 
        // "    linear z\t\t: " << car_status_msg.vehicle_status.acceleration.linear.z << "\n" <<
        // "    orientation x\t: " << car_status_msg.vehicle_status.orientation.x << "\n" <<
        // "    orientation y\t: " << car_status_msg.vehicle_status.orientation.y << "\n" <<
        // "    orientation z\t: " << car_status_msg.vehicle_status.orientation.z << "\n" <<
        // "    orientation w\t: " << car_status_msg.vehicle_status.orientation.w << "\n" <<
    std::endl;

    std::cout <<
        "  control" << "\n" <<
        "    throttle (0 ~ 1)\t: " << car_status_msg.vehicle_status.control.throttle << "\n" <<
        "    steer (-1 ~ 1)\t: " << car_status_msg.vehicle_status.control.steer << "\n" <<
        "    brake (0 ~ 1)\t: " << car_status_msg.vehicle_status.control.brake << "\n\n" <<       
        "  gear \t: " << car_status_msg.vehicle_status.control.gear << "\n" <<
    std::endl;


    std::cout <<
        "front wheel info" << "\n" <<
        "  position x\t: " << car_status_msg.front_wheel_pose.position.x << "\n" <<
        "  position y\t: " << car_status_msg.front_wheel_pose.position.y << "\n" <<
        "  position z\t: " << car_status_msg.front_wheel_pose.position.z << "\n" <<
        "  orientation x\t: " << car_status_msg.front_wheel_pose.orientation.x << "\n" <<
        "  orientation y\t: " << car_status_msg.front_wheel_pose.orientation.y << "\n" <<
        "  orientation z\t: " << car_status_msg.front_wheel_pose.orientation.z << "\n" <<
        "  orientation w\t: " << car_status_msg.front_wheel_pose.orientation.w << "\n" <<
    std::endl;

    std::cout << "==============================\n" << std::endl;
}
