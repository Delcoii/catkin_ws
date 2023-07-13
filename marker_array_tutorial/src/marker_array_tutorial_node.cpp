#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

int main (int argc, char** argv) {
    ros::init (argc, argv, "mark_arr");
    ros::NodeHandle nh;
    ros::Rate loop_rate_hz(1);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/marker_arr_tutorial", 100);
    // (publisher name, queue size)

    uint32_t marker_shape = visualization_msgs::Marker::ARROW;

    visualization_msgs::MarkerArray mark2pub;
    mark2pub.markers.resize(3);


    while (ros::ok()) {

        for (int i = 0 ; i < 3; i++) {
            mark2pub.markers[i].header.frame_id = "base_link";
            mark2pub.markers[i].header.stamp = ros::Time::now();

            mark2pub.markers[i].ns = "marker_arr";
            mark2pub.markers[i].id = i;

            mark2pub.markers[i].type = marker_shape;
            mark2pub.markers[i].action = visualization_msgs::Marker::ADD;
        }
        
        for (int i = 0; i < 3; i++) {
            mark2pub.markers[i].pose.position.x = (double)i*0.2;
            mark2pub.markers[i].pose.position.y = (double)i*0.2;
            mark2pub.markers[i].pose.position.z = (double)i*0.2;

            mark2pub.markers[i].pose.orientation.x = 0.0;
            mark2pub.markers[i].pose.orientation.y = 0.0;
            mark2pub.markers[i].pose.orientation.z = 0.0;
            mark2pub.markers[i].pose.orientation.w = 0.0;

            mark2pub.markers[i].scale.x = 0.1;
            mark2pub.markers[i].scale.y = 0.02;
            mark2pub.markers[i].scale.z = 0.02;

            mark2pub.markers[i].color.r = 0.0f;
            mark2pub.markers[i].color.g = 1.0f;
            mark2pub.markers[i].color.b = 0.0f;
            mark2pub.markers[i].color.a = 1.0f;

            mark2pub.markers[i].lifetime = ros::Duration();
        }
        
        // std::cout << "pub " << std::endl;
        
        marker_pub.publish(mark2pub);
        ROS_INFO("published\n");

        loop_rate_hz.sleep();
    }

    return 0;
}