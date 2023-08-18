#include "control_node/ControlMsgs/ControlMsgs.h"
#include "control_node/waypoint_save/waypoint_save.h"
#include "control_node/longitudinal_control/longitudinal_control.h"
#include "control_node/stanley/stanley.h"
#include "control_node/error_calculate/error_calculate.h"
#include "control_node/pure_pursuit/PurePursuitControl.h"

#define LOOP_HZ                 60.
#define P_GAIN                  10.
#define I_GAIN                  0.
#define D_GAIN                  1.
#define SWITCH_VEL_MS           15.
#define BRAKE_MAX               -0.2
// #define TARGET_VELOCITY_MS      20.


int main(int argc, char** argv) {

    std::vector<std::vector<double>> waypoints;
    GetWaypoints(waypoints);
    LatLon2Utm(waypoints);
    SetVelocityProfile(waypoints);

    ros::init(argc, argv, "control_node");
    ros::Time::init();
    ros::Rate loop_rate(LOOP_HZ);

    ControlMsgs msg4control;
    control_node::CarlaEgoVehicleStatus car_stat;
    geometry_msgs::PoseStamped front_wheel_pose;
    geometry_msgs::PoseStamped rear_wheel_pose;

    FollowingError error_check;

    PurePursuitControl pp;
    StanleyControl stanley;
    LongitudinalControl longi_control((1./LOOP_HZ), 1., BRAKE_MAX, P_GAIN, D_GAIN, I_GAIN);
    double throttle;
    double brake;
    double steer;
    
    stanley.GetAllWaypoints(waypoints);
    pp.GetAllWaypoints(waypoints);
    int visualizing_target_idx = 0;
    while (ros::ok()) {

        // not subscribed yet
        if (msg4control.FrPoseReceived() == false) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        
        // arrived.. node ends
        if (visualizing_target_idx >= waypoints.size()-5) {
            std::cout << error_check.cte_err_avg() << std::endl;
            msg4control.PubControlMsg(0., 0., 1.);
            return 0;
        }

        msg4control.SetValue(car_stat, front_wheel_pose, rear_wheel_pose);
        longi_control.SetThrottleBrake(waypoints[stanley.target_idx()][TARGET_VEL_IDX], car_stat.velocity);
        longi_control.SetVal(throttle, brake);


        stanley.SetSteer(front_wheel_pose, car_stat.velocity);
        pp.SetSteer(rear_wheel_pose, car_stat.velocity);
        if (car_stat.velocity < SWITCH_VEL_MS) {        // if car is slower than 15m/s
            
            std::cout << "using stanley controller\n";
            steer = stanley.steer_val();
    
            visualizing_target_idx = stanley.target_idx();
            msg4control.PubVisMsg(waypoints[visualizing_target_idx]);
            msg4control.PubControlMsg(throttle, steer, brake);
            // stanley.PrintValue();
        }
        else {                                          // if car is faster..
            std::cout << "using pure pursuit controller\n";
            steer = pp.steer_val();

            visualizing_target_idx = pp.target_idx();
            msg4control.PubVisMsg(waypoints[visualizing_target_idx]);
            msg4control.PubControlMsg(throttle, steer, brake);
            // pp.PrintValue();
        }

        visualizing_target_idx = stanley.target_idx();  // using just closest waypoint
        error_check.FilteringCTE(stanley.distance_m());
        error_check.GetSpeed(waypoints[visualizing_target_idx][TARGET_VEL_IDX], car_stat.velocity);
        error_check.PubError();           // publish cross track err
        
    

        
        std::cout <<
            "idx : " << visualizing_target_idx << "\n" << 
            "target vel(km/h) : " << waypoints[visualizing_target_idx][TARGET_VEL_IDX]*3.6 << "\n" <<
            "kappa : " << waypoints[visualizing_target_idx][CURVATURE_IDX] << "\n" <<
        std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}