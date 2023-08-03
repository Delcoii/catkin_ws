#include "pure_pursuit/ControlMsgs.h"
#include "pure_pursuit/waypoint_save.h"


int main(int argc, char** argv) {

    ros::init (argc, argv, "pure_pursuit_node");
    ros::Time::init();

    ControlMsgs control_msgs;

    // bagfile car odometry -> waypoints
    std::vector<std::vector<double>> waypoints;
    GetWaypoints(waypoints);
    



    return 0;
}