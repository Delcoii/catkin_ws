#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>



int main (int argc, char** argv) {

    std::ifstream ifstr_pose ("/home/delcoii/catkin_ws/src/my_car_waypoint_display/src/my_car_odometry_pose.txt");
    std::ifstream ifstr_twist ("/home/delcoii/catkin_ws/src/my_car_waypoint_display/src/my_car_odometry_twist.txt");

    // read pose position x y z, and orientation x y z w
    std::vector<std::vector<double>> wypt_pose;

    // read twist linear x y z and angular x y z
    std::vector<std::vector<double>> wypt_twist;

    // amount of displaying odometry in rviz
    int odometry_display_count = 0;  

    if (!ifstr_pose) {
        ROS_INFO("cannot find pose file!\n");
        return 0;
    }
    if (!ifstr_twist) {
        ROS_INFO("cannot find twist file!\n");
        return 0;
    }

    // get pose.position data x, y, z
    while (ifstr_pose) {
        std::vector<double> temp(7, 0);

        for (int i = 0; i < 7; i++) {
            ifstr_pose >> temp[i];
        }
        
        wypt_pose.push_back(temp);
        
        //initialize once
        odometry_display_count++;
    }

    // get pose.position data x, y, z
    while (ifstr_twist) {
        std::vector<double> temp(6, 0);

        for (int i = 0; i < 6; i++) {
            ifstr_twist >> temp[i];
        }
        
        wypt_twist.push_back(temp);
    }


    ros::init (argc, argv, "my_car_waypoint_publish");
    ros::NodeHandle nh;

    ros::Rate loop_rate_hz(30);

    ros::Publisher wypt_odo_pub = nh.advertise<nav_msgs::Odometry>("cybertruck_waypoint", 100);

    nav_msgs::Odometry odo2display;
    ros::Time now;
    int displayed_odometry_count = 0;

    while (ros::ok()) {

        now = ros::Time::now();

        odo2display.header.frame_id = "map";
        odo2display.header.stamp = now;


        odo2display.pose.pose.position.x = wypt_pose[displayed_odometry_count][0];
        odo2display.pose.pose.position.y = wypt_pose[displayed_odometry_count][1];
        odo2display.pose.pose.position.z = wypt_pose[displayed_odometry_count][2];
        

        odo2display.pose.pose.orientation.x = wypt_pose[displayed_odometry_count][3];
        odo2display.pose.pose.orientation.y = wypt_pose[displayed_odometry_count][4];
        odo2display.pose.pose.orientation.z = wypt_pose[displayed_odometry_count][5];
        odo2display.pose.pose.orientation.w = wypt_pose[displayed_odometry_count][6];


        odo2display.twist.twist.linear.x = wypt_twist[displayed_odometry_count][0];
        odo2display.twist.twist.linear.y = wypt_twist[displayed_odometry_count][1];
        odo2display.twist.twist.linear.z = wypt_twist[displayed_odometry_count][2];

        odo2display.twist.twist.angular.x = wypt_twist[displayed_odometry_count][3];
        odo2display.twist.twist.angular.y = wypt_twist[displayed_odometry_count][4];
        odo2display.twist.twist.angular.z = wypt_twist[displayed_odometry_count][5];

        displayed_odometry_count++;
        if (displayed_odometry_count >= odometry_display_count)
            displayed_odometry_count = 0;

        wypt_odo_pub.publish(odo2display);
        ROS_INFO("published odometry %d \n", displayed_odometry_count);

        loop_rate_hz.sleep();
    }

}