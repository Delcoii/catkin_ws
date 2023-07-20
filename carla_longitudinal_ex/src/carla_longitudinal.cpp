#include <ros/ros.h>
#include <carla_longitudinal_ex/CarlaEgoVehicleControl.h>
#include <carla_longitudinal_ex/CarlaEgoVehicleStatus.h>

#include "carla_longitudinal_ex/pid.h"


void CarStatusCallback (const carla_longitudinal_ex::CarlaEgoVehicleStatus::ConstPtr& msg);

class SubdCarData {
public:

    ros::NodeHandle nh;
    ros::Publisher throttle_pub;
    ros::Subscriber car_status_sub;

    double velocity;
    double throttle;
    
    SubdCarData() {
        car_status_sub = nh.subscribe("/carla/ego_vehicle/vehicle_status", 100, &SubdCarData::CarStatusCallback, this);

        throttle_pub = nh.advertise<carla_longitudinal_ex::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 100);
    }

    
    void CarStatusCallback(const carla_longitudinal_ex::CarlaEgoVehicleStatus::ConstPtr& msg) {
        // std::cout << "get!!" << std::endl;
        this->velocity = msg->velocity;
    }

};



int main(int argc, char** argv) {

    ros::init (argc, argv, "carla_longitudinal_ex");
    ros::Time::init();
    ros::Rate loop_rate_hz(60);

    SubdCarData car_data;
    carla_longitudinal_ex::CarlaEgoVehicleControl control_msg;

    PID throttle_control = PID((1./60.), 1., 0., 3., 0., 0.);

    double throttle_out = 0.5;
    double pid_output;
    while (ros::ok()) {

        control_msg.header.stamp = ros::Time::now();


        pid_output = throttle_control.calculate(10., car_data.velocity);
        std::cout << "velocity(m/s) : " << car_data.velocity << std::endl;
        std::cout << "PID output : " << pid_output << std::endl;
      
        control_msg.throttle = pid_output;
        std::cout << "output throttle : " << control_msg.throttle << std::endl;
        car_data.throttle_pub.publish(control_msg);

        loop_rate_hz.sleep();
        ros::spinOnce();
    }

    return 0;
}

