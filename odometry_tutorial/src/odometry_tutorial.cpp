#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main (int argc, char** argv) {

    ros::init (argc, argv, "odo_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate_10hz(1);

    ros::Publisher odo_pub = nh.advertise<nav_msgs::Odometry>("odo_publish", 100);
    tf::TransformBroadcaster odom_broadcaster;


    nav_msgs::Odometry odo2display;
    ros::Time now;

    double pose_x = 0.0, pose_y = 0.0, pose_z = 0.0;

    while (ros::ok()) {
        
        now = ros::Time::now();
        
        odo2display.header.frame_id = "base";
        odo2display.header.stamp = now;    

        if (pose_x > 10.0) {
            pose_x = 0.0;
            pose_y = 0.0;
            pose_z = 0.0;
        }
        else {
            pose_x += 0.1;
            pose_y += 0.1;
            pose_z += 0.1;
        }

        odo2display.pose.pose.position.x = pose_x;
        odo2display.pose.pose.position.y = pose_y;
        odo2display.pose.pose.position.z = pose_z;

        
        odo2display.pose.pose.orientation.x = 0.0;
        odo2display.pose.pose.orientation.y = 0.0;
        odo2display.pose.pose.orientation.z = 0.0;
        odo2display.pose.pose.orientation.w = 0.0;

        odo2display.twist.twist.linear.x = 0.0;
        odo2display.twist.twist.linear.y = 0.0;
        odo2display.twist.twist.linear.z = 0.0;

        odo2display.twist.twist.angular.x = 0.0;
        odo2display.twist.twist.angular.y = 0.0;
        odo2display.twist.twist.angular.z = 0.0;


        odo_pub.publish(odo2display);

        loop_rate_10hz.sleep();
    }

}