#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");
    ros::NodeHandle nh;

    ros::Publisher tf_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("front_wheel_pub", 100);
    tf::TransformListener listener;

    geometry_msgs::PoseStamped pose2pub;

    ros::Rate rate(60.0);
    
    while (ros::ok()) {

        tf::StampedTransform transform;
        try {
            listener.lookupTransform("map", "front_wheel_center", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        std::cout << transform.getOrigin().x() << std::endl;
        std::cout << transform.getOrigin().y() << std::endl;
        std::cout << transform.getOrigin().z() << std::endl;
        std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n" << std::endl;
        rate.sleep();
    }
    return 0;
};