#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv) {

    ros::init (argc, argv, "wheel_center_tf");
    ros::NodeHandle nh;

    ros::Rate loop_rate_hz(60.);

    tf::TransformBroadcaster trsfm_brdcster;
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
        

        loop_rate_hz.sleep();
    }

    return 0;
}