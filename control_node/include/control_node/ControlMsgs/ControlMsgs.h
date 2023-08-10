#ifndef __CONTROL_NODE_H__
#define __CONTROL_NODE_H__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include "control_node/CarlaEgoVehicleStatus.h"
#include "control_node/CarlaEgoVehicleControl.h"


#define CUBE_SIZE           0.5
#define CUBE_COLOR_R        1.0f
#define CUBE_COLOR_G        0.8f
#define CUBE_COLOR_B        0.5f
#define CUBE_COLOR_A        1.0f    // orange


class ControlMsgs {

    ros::NodeHandle nh;
    
    ros::Subscriber vehicle_status_sub;
    control_node::CarlaEgoVehicleStatus vehicle_status;

    ros::Subscriber front_wheel_sub;
    ros::Subscriber rear_wheel_sub;
    geometry_msgs::PoseStamped fr_pose;
    geometry_msgs::PoseStamped rr_pose;
    bool fr_pose_received;
    bool rr_pose_received;

    ros::Publisher control_pub;
    control_node::CarlaEgoVehicleControl ctrl_pub_msg;

    ros::Publisher target_vis_pub;
    visualization_msgs::Marker target_wypt_marker;

public:

    ControlMsgs ();
    bool FrPoseReceived();
    
    void GetCarStatus(const control_node::CarlaEgoVehicleStatus::ConstPtr& msg);
    void GetFrontWheelPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void GetRearWheelPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void SetValue(control_node::CarlaEgoVehicleStatus& _car_stat, geometry_msgs::PoseStamped& _fr_pose, geometry_msgs::PoseStamped& _rr_pose);

    void PubControlMsg(double throttle, double steer, double brake);
    void PubVisMsg(std::vector<double> wypt);


};






#endif // __CONTROL_NODE_H__