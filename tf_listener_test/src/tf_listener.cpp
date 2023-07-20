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
    
    double pose_x, pose_y, pose_z;

    while (ros::ok()) {

        tf::StampedTransform transform;
        try {
            listener.lookupTransform("map", "front_wheel_center", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        pose_x = transform.getOrigin().x();
        pose_y = transform.getOrigin().y();
        pose_z = transform.getOrigin().z();

        /*
        std::cout <<
            "listening and calculating \n" <<
            "position x : "<< pose_x << "\n" << 
            "position y : "<< pose_y << "\n" <<
            "position z : "<< pose_z << "\n" <<
            "=========\n" <<
        std::endl;
        */

        pose2pub.header.stamp = ros::Time::now();
        pose2pub.pose.position.x = pose_x;
        pose2pub.pose.position.y = pose_y;
        pose2pub.pose.position.z = pose_z;

        tf_pose_pub.publish(pose2pub);

        rate.sleep();
    }
    return 0;
};