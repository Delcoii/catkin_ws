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
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <armadillo>

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_projection/UTM.h"


#define POSITION_X              0
#define POSITION_Y              1
#define POSITION_Z              2
#define ORIENTATION_X           3
#define ORIENTATION_Y           4
#define ORIENTATION_Z           5
#define ORIENTATION_W           6
#define SPEED_MS                7

#define LATITUDE_IDX            0
#define LONGITUDE_IDX           1
#define REF_LONGITUDE           0.
#define REF_LATITUDE            0.


#define WYPT_DIST_M             0.2

// file reading func
void GetWaypoints(std::vector<std::vector<double>>& container);

// return data's vector length
int CSV2Data(std::string csv_location, std::string  reading_col_start_title, std::string reading_col_end_title, std::vector<std::vector<double>> &data_vec);

void WaypointRearrange (std::vector<std::vector<double>> &data_vec);

void LatLon2Utm(std::vector<std::vector<double>>& wytps);

void VelocityProfile(std::vector<std::vector<double>>& container);

#endif // __WAYPOINT_SAVE_H__