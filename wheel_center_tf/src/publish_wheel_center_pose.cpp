#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {

    ros::init (argc, argv, "wheel_center_tf");
    ros::NodeHandle nh;
    ros::Publisher fr_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("front_wheel_pose", 100);
    ros::Publisher rr_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("rear_wheel_pose", 100);

    ros::Rate loop_rate_hz(60.);

    geometry_msgs::PoseStamped fr_pose;
    geometry_msgs::PoseStamped rr_pose;

    tf::TransformBroadcaster trsfm_brdcster;
    tf::TransformListener wheel_tf_listener;
    tf::Transform front_wheel_trsfm;
    tf::Transform rear_wheel_trsfm;

    double fr_wheel_pos_x;
    double rr_wheel_pos_x;
    
    // read param from launch file
    // ~ needed to get local namespace parameter
    
    // default ford mustang generation1
    nh.param<double>("front_wheel_pos_x", fr_wheel_pos_x, 1.5617858509105815);
    nh.param<double>("rear_wheel_pos_x", rr_wheel_pos_x, -1.3244055556930903);

    front_wheel_trsfm.setOrigin (tf::Vector3(fr_wheel_pos_x, 0.0, 0.0));
    front_wheel_trsfm.setRotation(tf::Quaternion(0., 0., 0., 1.));

    rear_wheel_trsfm.setOrigin (tf::Vector3(rr_wheel_pos_x, 0.0, 0.0));
    rear_wheel_trsfm.setRotation(tf::Quaternion(0., 0., 0., 1.));


    while (ros::ok()) {

        ros::Time current_time = ros::Time::now();

        trsfm_brdcster.sendTransform(tf::StampedTransform(front_wheel_trsfm, current_time, "ego_vehicle", "front_wheel_center"));
        trsfm_brdcster.sendTransform(tf::StampedTransform(rear_wheel_trsfm, current_time, "ego_vehicle", "rear_wheel_center"));


        tf::StampedTransform fr_tf2pub;
        tf::StampedTransform rr_tf2pub;
        try {
            wheel_tf_listener.lookupTransform("map", "front_wheel_center", ros::Time(0), fr_tf2pub);
            wheel_tf_listener.lookupTransform("map", "rear_wheel_center", ros::Time(0), rr_tf2pub);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        
        fr_pose.header.stamp = ros::Time::now();
        fr_pose.pose.position.x = fr_tf2pub.getOrigin().x();
        fr_pose.pose.position.y = fr_tf2pub.getOrigin().y();
        fr_pose.pose.position.z = fr_tf2pub.getOrigin().z();
        fr_pose.pose.orientation.x = fr_tf2pub.getRotation().x();
        fr_pose.pose.orientation.y = fr_tf2pub.getRotation().y();
        fr_pose.pose.orientation.z = fr_tf2pub.getRotation().z();
        fr_pose.pose.orientation.w = fr_tf2pub.getRotation().w();


        rr_pose.header.stamp = ros::Time::now();
        rr_pose.pose.position.x = rr_tf2pub.getOrigin().x();
        rr_pose.pose.position.y = rr_tf2pub.getOrigin().y();
        rr_pose.pose.position.z = rr_tf2pub.getOrigin().z();
        rr_pose.pose.orientation.x = rr_tf2pub.getRotation().x();
        rr_pose.pose.orientation.y = rr_tf2pub.getRotation().y();
        rr_pose.pose.orientation.z = rr_tf2pub.getRotation().z();
        rr_pose.pose.orientation.w = rr_tf2pub.getRotation().w();

        fr_pose_pub.publish(fr_pose);
        rr_pose_pub.publish(rr_pose);

        loop_rate_hz.sleep();
    }

    return 0;
}