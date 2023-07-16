#include <string>
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <carla_callback_vehicle_status/CarlaEgoVehicleControl.h>
#include <carla_callback_vehicle_status/CarlaEgoVehicleStatus.h>


void CarStatusCallback (const carla_callback_vehicle_status::CarlaEgoVehicleStatus::ConstPtr& msg) {
    
    // for printing bool values
    std::string hand_brake_print;
    std::string reverse_print;
    std::string manual_gear_print;
    
    // for conversion, quaternion to euler 
    tf::Quaternion car_quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    
    tf::Matrix3x3 car_mat(car_quat);
    double roll, pitch, yaw;
    car_mat.getRPY(roll, pitch, yaw);



    if (msg->control.hand_brake == true)
        hand_brake_print = "true";
    else
        hand_brake_print = "false";
    
    if (msg->control.reverse == true)
        reverse_print = "true";
    else
        reverse_print = "false"; 

    if (msg->control.manual_gear_shift == true)
        manual_gear_print = "true";
    else
        manual_gear_print = "false";


    std::cout <<
        "seq : " << msg->header.seq << "\n" << 
        "stamp sec : " << msg->header.stamp.sec << "\n" << 
        "frame_id : " << msg->header.frame_id << "\n" << 
    std::endl;
    
    std::cout << 
        "velocity(m/s) \t: " << msg->velocity << "\n\n" << 
        "acceleration (m/s^2): " << "\n" <<
        "linear x\t\t: " << msg->acceleration.linear.x << "\n" <<
        "linear y\t\t: " << msg->acceleration.linear.y << "\n" << 
        "linear z\t\t: " << msg->acceleration.linear.z << "\n\n" <<

        "orientation x\t\t: " << msg->orientation.x << "\n" <<
        "orientation y\t\t: " << msg->orientation.y << "\n" <<
        "orientation z\t\t: " << msg->orientation.z << "\n" <<
        "orientation w\t\t: " << msg->orientation.w << "\n" <<
        "roll \t\t\t: " << roll << "\n" <<
        "pitch \t\t\t: " << pitch << "\n" << 
        "yaw \t\t\t: " << yaw << "\n" <<
        "roll(deg) \t\t: " << roll*180./M_PI << "\n" << 
        "pitch(deg) \t\t: " << pitch*180./M_PI << "\n" << 
        "yaw(deg) \t\t: " << yaw*180./M_PI << "\n"
    << std::endl;

    std::cout <<
        "control : " << "\n" <<
        "throttle (0 ~ 1)\t: " << msg->control.throttle << "\n" <<
        "steer (-1 ~ 1)\t\t: " << msg->control.steer << "\n" <<
        "brake (0 ~ 1)\t\t: " << msg->control.brake << "\n" <<
    std::endl;

    std::cout << 
        "(bool)\n" << 
        "hand_brake \t\t:" << hand_brake_print << "\n" <<
        "reverse \t\t:" << reverse_print << "\n" <<
        "manual_gear_shift \t:" << manual_gear_print << "\n\n" <<
        
        "(-1 ~ 6)\ngear \t: " << msg->control.gear << "\n" <<
    std::endl;

    std::cout << "==============================\n" << std::endl;
}


int main(int argc, char** argv) {

    ros::init (argc, argv, "status_callback_node");
    ros::NodeHandle nh;


    ros::Subscriber status_rosinfo_sub = nh.subscribe("/carla/ego_vehicle/vehicle_status", 100, CarStatusCallback);

    ros::spin();

    return 0;
}