#include "control_node/ControlMsgs/ControlMsgs.h"
#include "control_node/waypoint_save/waypoint_save.h"
#include "control_node/longitudinal_control/longitudinal_control.h"
#include "control_node/stanley/stanley.h"
#include "control_node/error_calculate/error_calculate.h"
#include "control_node/pure_pursuit/PurePursuitControl.h"

#define LOOP_HZ                 60.
#define P_GAIN                  10.
#define I_GAIN                  0.
#define D_GAIN                  0.
#define SWITCH_VEL_MS           15.
// #define TARGET_VELOCITY_MS      20.


int main(int argc, char** argv) {
    ros::init(argc, argv, "control_node");
    ros::Time::init();
    ros::Rate loop_rate(LOOP_HZ);

    std::vector<std::vector<double>> waypoints;
    GetWaypoints(waypoints);
    LatLon2Utm(waypoints);
    SetVelocityProfile(waypoints);

    ControlMsgs msg4control;
    control_node::CarlaEgoVehicleStatus car_stat;
    geometry_msgs::PoseStamped front_wheel_pose;
    geometry_msgs::PoseStamped rear_wheel_pose;

    FollowingError error_check;

    PurePursuitControl pp;
    StanleyControl stanley;
    LongitudinalControl longi_control((1./LOOP_HZ), 1., -1., P_GAIN, D_GAIN, I_GAIN);
    double throttle;
    double brake;
    double steer;
    
    stanley.GetAllWaypoints(waypoints);
    pp.GetAllWaypoints(waypoints);
    int visualizing_target_idx = 0;
    while (ros::ok()) {

    
        if (msg4control.FrPoseReceived() == false) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        

        if (visualizing_target_idx >= waypoints.size()-5) {
            std::cout << error_check.err_avg() << std::endl;
            return 0;
        }

        msg4control.SetValue(car_stat, front_wheel_pose, rear_wheel_pose);
        longi_control.SetThrottleBrake(waypoints[stanley.target_idx()][TARGET_VEL_IDX], car_stat.velocity);
        longi_control.SetVal(throttle, brake);


        stanley.SetSteer(front_wheel_pose, car_stat.velocity);
        pp.SetSteer(rear_wheel_pose, car_stat.velocity);
        if (car_stat.velocity < SWITCH_VEL_MS) {      // if car is slower than 15m/s
            
            std::cout << "using stanley controller\n";
            steer = stanley.steer_val();
            visualizing_target_idx = stanley.target_idx();
            msg4control.PubVisMsg(waypoints[visualizing_target_idx]);
            stanley.PrintValue();
        }
        else {                              // if car is faster..
            
            std::cout << "using pure pursuit controller\n";
            steer = pp.steer_val();
            visualizing_target_idx = pp.target_idx();
            msg4control.PubVisMsg(waypoints[visualizing_target_idx]);
            pp.PrintValue();
        }
    
        
        msg4control.PubControlMsg(throttle, steer, brake);


        visualizing_target_idx = stanley.target_idx();
        std::cout <<
            "idx : " << visualizing_target_idx << "\n" << 
            "target vel(km/h) : " << waypoints[visualizing_target_idx][TARGET_VEL_IDX]*3.6 << "\n" <<
            "kappa : " << waypoints[visualizing_target_idx][CURVATURE_IDX] << "\n" <<
        std::endl;

        
        // use distance from stanley
        error_check.FilteredValue(stanley.distance_m());
        error_check.PubCrossTrackError();           // publish cross track err

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}