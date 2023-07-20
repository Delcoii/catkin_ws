/* get waypoints pose value
 * idx 0 : position x
 * idx 1 : position y
 * idx 2 : position z
 * idx 3 : orientation x
 * idx 4 : orientation y
 * idx 5 : orientation z
 * idx 6 : orientation w
*/

#ifndef __WAYPOINT_SAVE_H__
#define __WAYPOINT_SAVE_H__

#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>


#define POSE_X              0
#define POSE_Y              1
#define POSE_Z              2
#define ORIENTATION_X       3
#define ORIENTATION_Y       4
#define ORIENTATION_Z       5
#define ORIENTATION_W       6

#define WYPT_DIST           0.2


void SaveWaypoint(std::vector<std::vector<double>>& vec_arr);


#endif // __WAYPOINT_SAVE_H__