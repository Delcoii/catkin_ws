#include "control_node/ControlMsgs/ControlMsgs.h"
#include "control_node/waypoint_save/waypoint_save.h"
#include "control_node/pid/pid.h"
#include "control_node/stanley/stanley.h"
#include "control_node/error_calculate/error_calculate.h"
#include "control_node/pure_pursuit/PurePursuitControl.h"

#define LOOP_HZ                 60.
#define P_GAIN                  3.
#define I_GAIN                  0.
#define D_GAIN                  1.
#define TARGET_VELOCITY_MS      20.


int main(int argc, char** argv) {
    ros::init(argc, argv, "control_node");
    ros::Time::init();
    ros::Rate loop_rate(LOOP_HZ);

    std::vector<std::vector<double>> waypoints;
    GetWaypoints(waypoints);
    // LatLon2Utm(waypoints);
    WaypointRearrange(waypoints);

    ControlMsgs msg4control;
    control_node::CarlaEgoVehicleStatus car_stat;
    geometry_msgs::PoseStamped front_wheel_pose;
    geometry_msgs::PoseStamped rear_wheel_pose;

    FollowingError error_check;

    PurePursuitControl pp;
    StanleyControl stanley;
    PID longi_control = PID((1./LOOP_HZ), 1., 0., P_GAIN, D_GAIN, I_GAIN);
    double throttle;
    double steer;
    
    stanley.GetAllWaypoints(waypoints);
    pp.GetAllWaypoints(waypoints);
    int visualizing_target_idx;
    while (ros::ok()) {
        if (visualizing_target_idx == waypoints.size()-1) {
            std::cout << error_check.err_avg() << std::endl;
            return 0;
        }
        
        if (msg4control.FrPoseReceived() == false) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        msg4control.SetValue(car_stat, front_wheel_pose, rear_wheel_pose);


        throttle = longi_control.calculate(TARGET_VELOCITY_MS, car_stat.velocity);


        stanley.SetSteer(front_wheel_pose, car_stat.velocity);
        pp.SetSteer(rear_wheel_pose);
        
            
        std::cout << "using pure pursuit controller\n";
        steer = pp.steer_val();
        visualizing_target_idx = pp.target_idx();
        msg4control.PubVisMsg(waypoints[visualizing_target_idx]);
    

        // msg4control.PubControlMsg(throttle, steer, 0.);
        msg4control.PubControlMsg(throttle, 0., 0.);

        std::cout << visualizing_target_idx << std::endl;
        // stanley.PrintValue();
        // pp.PrintValue();

        // use distance from stanley
        error_check.FilteredValue(stanley.distance_m());
        error_check.PubCrossTrackError();   // publish cross track err


        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}