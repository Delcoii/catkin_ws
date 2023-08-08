#include "waypoint_save/waypoint_save.h"
#include "curve_display/display.h"

#include <ros/ros.h>



int main(int argc, char** argv) {

    std::vector<std::vector<double>> waypoints;

    GetWaypoints(waypoints);
    LatLon2Utm(waypoints);

    ros::init(argc, argv, "pseudo_inverse");
    ros::Time::init();
    ros::Rate loop_rate_hz(60.);

    CurveDisplayer curve;
    int target_wypt_idx = 0;

    while (ros::ok()) {
        
        // if nan initialized in x_mat
        if (curve.SetMatrixCols(waypoints, target_wypt_idx) == false) {
            ros::spinOnce();
            loop_rate_hz.sleep();
            continue;
        }

        curve.GetCarWyptDist(
            waypoints[target_wypt_idx][0],
            waypoints[target_wypt_idx][1]
        );

        curve.PrintInfo();
        // compare to lookahead dist and update
        curve.CheckNextTarget(target_wypt_idx);

        curve.GetMatA();
        curve.SetMarker();
        curve.Visualize();      // publish markerarray

        
        std::cout << "\ntarget waypoint : " << target_wypt_idx << std::endl;

        ros::spinOnce();
        loop_rate_hz.sleep();
    }

}