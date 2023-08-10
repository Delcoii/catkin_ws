/*
 * todo list
 * 1. control by gnss data
 * 2. calculate heading by....wha??
 */


#include "pure_pursuit/waypoint_save/waypoint_save.h"
#include "pure_pursuit/ControlMsgs/ControlMsgs.h"
#include "pure_pursuit/PurePursuitControl.h"
#include "pure_pursuit/pid/pid.h"

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_projection/UTM.h"


#define P_GAIN          3.
#define I_GAIN          0.
#define D_GAIN          1.

#define REF_LATITUDE    0.
#define REF_LONGITUDE   0.
#define TARGET_VEL_MS   10.


int main(int argc, char** argv) {

    ros::init (argc, argv, "following_lat_lon");
    ros::Time::init();
    ros::Rate loop_rate_hz(60.0);

    // bagfile car odometry -> waypoints
    std::vector<std::vector<double>> waypoints;
    GetWaypoints(waypoints);


    // utm projection
    lanelet::Origin origin({REF_LATITUDE, REF_LONGITUDE});
    auto projector = lanelet::projection::UtmProjector(origin);
    for (int idx = 0; idx < waypoints.size(); idx++) {
        lanelet::BasicPoint3d utm_coords = projector.forward(
            {waypoints[idx][POSITION_X],
            waypoints[idx][POSITION_Y],
            0}
        );

        waypoints[idx][POSITION_X] = utm_coords.x();
        waypoints[idx][POSITION_Y] = utm_coords.y();
    }
    std::cout << "projection complete, waypoint size : " << waypoints.size() << std::endl;


    // subscribe and publise
    ControlMsgs msgs;

    PID longitudinal_pid = PID((1./60.), 1., 0., P_GAIN, D_GAIN, I_GAIN);
    double throttle_pid;

    // pure pursuit calculation
    PurePursuitControl pp_ctr;
    int target_wypt_idx = 0;
    double steer_output;

    while(ros::ok()) {

        throttle_pid = longitudinal_pid.calculate(TARGET_VEL_MS, msgs.vehicle_status.velocity);

        steer_output = pp_ctr.SetSteer(
            waypoints[target_wypt_idx][POSITION_X],
            waypoints[target_wypt_idx][POSITION_Y]
        );


        std::cout << 
            "target waypoint index : " << target_wypt_idx << "\n" <<
            "waypoint x : " << waypoints[target_wypt_idx][POSITION_X] << "\n" <<
            "waypoint y : " << waypoints[target_wypt_idx][POSITION_Y] << "\n" <<
            "steer : " << steer_output << "\n" << 
            "==================================================\n" <<
        std::endl;

        msgs.PubMsg(throttle_pid, steer_output, 0.);

        if (target_wypt_idx == waypoints.size()-1) {
            return 0;
        }

        if (pp_ctr.ArrivedLKDist() == true) {
            target_wypt_idx++;
        }
        
        pp_ctr.print_val();

        ros::spinOnce();
        loop_rate_hz.sleep();
    }


    return 0;
}