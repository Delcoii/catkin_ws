#include <ros/ros.h>
// #include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {

    ros::init (argc, argv, "add_stanley_tf");
    ros::NodeHandle nh;
    // ros::Publisher stanley_tf_pub = nh.advertise<tf::tfMessage>("stanley_tf", 100);

    ros::Rate loop_rate_hz(60);

    tf::TransformBroadcaster trsfm_brdcster;
    tf::Transform trsfm;
    
    // geometry_msgs::TransformStamped trsfm2pub;

    ros::Time current_time;

    while (ros::ok()) {

        current_time = ros::Time::now();

        // trsfm2pub.header.frame_id = "map";
        // trsfm2pub.header.stamp = current_time;

        trsfm.setOrigin (tf::Vector3(1.9035, 0.0, 0.0));
        trsfm.setRotation(tf::Quaternion(0, 0, 0, 1));

        trsfm_brdcster.sendTransform(tf::StampedTransform(trsfm, current_time, "ego_vehicle", "front_wheel_center"));


        loop_rate_hz.sleep();
    }

    return 0;
}