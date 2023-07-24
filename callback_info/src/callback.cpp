#include <string>
#include <iostream>

#include "callback_info/car_status.h"

int main(int argc, char** argv) {

    ros::init (argc, argv, "callback");
    ros::Time::init();
    ros::Rate loop_rate(60.);

    SubPubCarData sub_data_pub;

    tf::TransformListener listener;

    while (ros::ok()) {

        tf::StampedTransform front_wheel_tf;
        try {
            listener.lookupTransform("map", "front_wheel_center", ros::Time(0), front_wheel_tf);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        sub_data_pub.SetFrontWheelPose(front_wheel_tf);

        sub_data_pub.print_status();
        sub_data_pub.publish_data();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}