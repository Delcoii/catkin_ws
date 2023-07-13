#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


int main (int argc, char** argv) {

    std::ifstream ifstr_position ("/home/delcoii/catkin_ws/src/waypoint_odo/src/odo_pose_position_xyz.txt");
    std::ifstream ifstr_orient ("/home/delcoii/catkin_ws/src/waypoint_odo/src/odo_pose_orient_xyz.txt");

    std::vector<std::vector<double>> wypt_position_val;
    std::vector<std::vector<double>> wypt_orient_val;
    
    // amount of displaying odometry in rviz
    int odometry_display_count = 0;  

    if (!ifstr_position) {
        ROS_INFO("cannot find position file!\n");
        return 0;
    }
    if (!ifstr_orient) {
        ROS_INFO("cannot find orientation file!\n");
        return 0;
    }

    // get pose.position data x, y, z
    while (ifstr_position) {
        std::vector<double> temp_xyz (3, 0);

        for (int i = 0; i < 3; i++) {
            ifstr_position >> temp_xyz[i];
        }
        
        wypt_position_val.push_back(temp_xyz);
        
        //initialize once
        odometry_display_count++;
    }

    // get pose.position data x, y, z
    while (ifstr_orient) {
        std::vector<double> temp_xyz (4, 0);

        for (int i = 0; i < 4; i++) {
            ifstr_orient >> temp_xyz[i];
        }
        
        wypt_orient_val.push_back(temp_xyz);
    }


    ros::init (argc, argv, "odo_pub_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate_hz(20);

    ros::Publisher wypt_odo_pub = nh.advertise<nav_msgs::Odometry>("waypoint_odometry_publish", 100);

    nav_msgs::Odometry odo2display;
    ros::Time now;
    int displayed_odometry_count = 0;

    while (ros::ok()) {

        now = ros::Time::now();

        odo2display.header.frame_id = "test_frame";
        odo2display.header.stamp = now;


        odo2display.pose.pose.position.x = wypt_position_val[displayed_odometry_count][0];
        odo2display.pose.pose.position.y = wypt_position_val[displayed_odometry_count][1];
        odo2display.pose.pose.position.z = wypt_position_val[displayed_odometry_count][2];
        

        odo2display.pose.pose.orientation.x = wypt_orient_val[displayed_odometry_count][0];
        odo2display.pose.pose.orientation.y = wypt_orient_val[displayed_odometry_count][1];
        odo2display.pose.pose.orientation.z = wypt_orient_val[displayed_odometry_count][2];
        odo2display.pose.pose.orientation.w = wypt_orient_val[displayed_odometry_count][3];


        odo2display.twist.twist.linear.x = 0.0;
        odo2display.twist.twist.linear.y = 0.0;
        odo2display.twist.twist.linear.z = 0.0;

        odo2display.twist.twist.angular.x = 0.0;
        odo2display.twist.twist.angular.y = 0.0;
        odo2display.twist.twist.angular.z = 0.0;

        displayed_odometry_count++;
        if (displayed_odometry_count >= odometry_display_count)
            displayed_odometry_count = 0;

        wypt_odo_pub.publish(odo2display);
        ROS_INFO("published %d arrow\n", displayed_odometry_count);

        loop_rate_hz.sleep();
    }

}