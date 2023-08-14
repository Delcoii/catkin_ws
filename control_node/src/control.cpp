#include "control_node/ControlMsgs/ControlMsgs.h"
#include "control_node/waypoint_save/waypoint_save.h"
#include "control_node/pid/pid.h"
#include "control_node/stanley/stanley.h"
#include "control_node/error_calculate/error_calculate.h"

#define LOOP_HZ                 60.
#define P_GAIN                  3.
#define I_GAIN                  0.
#define D_GAIN                  1.
#define TARGET_VELOCITY_MS      10.


int main(int argc, char** argv) {
    ros::init(argc, argv, "control_node");
    ros::Time::init();
    ros::Rate loop_rate(LOOP_HZ);

    std::vector<std::vector<double>> waypoints;
    GetWaypoints(waypoints);
    LatLon2Utm(waypoints);

    ControlMsgs msg4control;
    control_node::CarlaEgoVehicleStatus car_stat;
    geometry_msgs::PoseStamped front_wheel_pose;
    geometry_msgs::PoseStamped rear_wheel_pose;

    FollowingError error_check;

    StanleyControl stanley;
    PID longi_control = PID((1./LOOP_HZ), 1., 0., P_GAIN, D_GAIN, I_GAIN);
    double throttle;
    double steer;
    
    stanley.GetAllWaypoints(waypoints);
    while (ros::ok()) {
        
        if (msg4control.FrPoseReceived() == false) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        msg4control.SetValue(car_stat, front_wheel_pose, rear_wheel_pose);


        throttle = longi_control.calculate(TARGET_VELOCITY_MS, car_stat.velocity);


        steer = stanley.GetSteeringValue(front_wheel_pose, car_stat.velocity);
        

        msg4control.PubControlMsg(throttle, steer, 0.);



        stanley.PrintValue();
        int target = stanley.TargetWaypointIdx();
        msg4control.PubVisMsg(waypoints[target]);   // visualize
        // use distance from stanley
        error_check.FilteredValue(stanley.distance_m());
        error_check.PubCrossTrackError();           // publish cross track err

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}