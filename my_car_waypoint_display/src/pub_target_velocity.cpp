#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "my_car_waypoint_display/waypoint_save/waypoint_save.h"

int main(int argc, char** argv) {

    std::vector<std::vector<double>> waypoints;
    GetWaypoints(waypoints);
    LatLon2Utm(waypoints);
    SetVelocityProfile(waypoints);

    ros::init(argc, argv, "target_velocity");
    ros::Time::init();
    ros::Rate loop_rate_hz(120.);

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("pub_target_velocity", 100);
    ros::Publisher pub_kappa = nh.advertise<std_msgs::Float64>("pub_kappa", 100);

    std::cout << "waiting 5 sec.." << std::endl;
    ros::Duration duration(5.0);
    duration.sleep();

    int idx = 0;
    std_msgs::Float64 vel;
    std_msgs::Float64 kappa;
    while(ros::ok()) {
        vel.data = waypoints[idx][TARGET_VEL_IDX];
        kappa.data = waypoints[idx][CURVATURE_IDX];
        pub.publish(vel);
        pub_kappa.publish(kappa);

        idx++;
        std::cout << idx << std::endl;
        if (idx >= waypoints.size()-1) {
            return 0;
        }

        loop_rate_hz.sleep();
    }
}