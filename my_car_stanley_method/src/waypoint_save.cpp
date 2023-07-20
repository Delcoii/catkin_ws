#include "my_car_stanley_method/waypoint_save.h"


void SaveWaypoint(std::vector<std::vector<double>>& vec_arr) {
    std::ifstream ifstr_pose ("/home/delcoii/catkin_ws/src/my_car_stanley_method/src/my_car_odometry_pose.txt");

    if (!ifstr_pose) {
        ROS_INFO("cannot find pose file!\n");
        return;
    }


    // get pose.position data x, y, z
    while (ifstr_pose) {
        std::vector<double> temp(7, 0);

        for (int i = 0; i < 7; i++) {
            ifstr_pose >> temp[i];
        }
        
        vec_arr.push_back(temp);
    }


    std::vector<double>prev_pose = vec_arr[0];

    for (int i = 1; i < vec_arr.size()-1; i++) {

        std::vector<double> temp = vec_arr[i];

        // calculate distance of each waypoints
        double distance = sqrt(pow(prev_pose[POSE_X]-temp[POSE_X], 2) + pow(prev_pose[POSE_Y]-temp[POSE_Y], 2));
        std::cout << distance << std::endl;

        // waypoint closer than 0.2m to previous one
        if (distance < WYPT_DIST) {
            vec_arr.erase(vec_arr.begin() + i);
            // std::cout << "erased!" << std::endl;
        }
        else {  // distance is farther than 0.19m
            prev_pose = vec_arr[i];
        }

    }

}