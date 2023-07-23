#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");
    ros::NodeHandle nh;

    ros::Publisher tf_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("front_wheel_pub", 100);
    tf::TransformListener listener;

    geometry_msgs::PoseStamped pose2pub;

    ros::Rate rate(60);
    
    double position_x, position_y, position_z;
    double orientation_x, orientation_y, orientation_z, orientation_w;

    while (ros::ok()) {

        tf::StampedTransform transform;
        try {
            listener.lookupTransform("map", "front_wheel_center", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        position_x = transform.getOrigin().x();
        position_y = transform.getOrigin().y();
        position_z = transform.getOrigin().z();
        orientation_x = transform.getRotation().x();
        orientation_x = transform.getRotation().y();
        orientation_x = transform.getRotation().z();
        orientation_x = transform.getRotation().w();


        pose2pub.header.stamp = ros::Time::now();
        pose2pub.pose.position.x = position_x;
        pose2pub.pose.position.y = position_y;
        pose2pub.pose.position.z = position_z;
        pose2pub.pose.orientation.x = orientation_x;
        pose2pub.pose.orientation.y = orientation_y;
        pose2pub.pose.orientation.z = orientation_z;
        pose2pub.pose.orientation.w = orientation_w;


        tf_pose_pub.publish(pose2pub);

        rate.sleep();
    }
    return 0;
};