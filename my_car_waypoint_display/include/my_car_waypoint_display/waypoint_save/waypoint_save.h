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

#include "my_car_waypoint_display/Filters/Filters.h"
#include "my_car_waypoint_display/math_functions/math_functions.h"

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


#define LOCAL_X                 0
#define LOCAL_Y                 1
#define TARGET_VEL_IDX          2
#define CURVATURE_IDX           3

#define MAX_LATERAL_ACCEL_MS2   3.6
#define HIGHWAY_SPEED_MS        41.6        // 150km/h
#define TOWN_SPEED_MS           9.7222      // 35km/h
#define ENTERING_HIGHWAY_IDX    4365
#define IDX_DIFF                20

#define SPEED_AVG_WINDOW_SIZE   20
#define KAPPA_AVG_WINDOW_SIZE   400
#define GAUSSIAN_WINDOW_SIZE    100

// file reading func
void GetWaypoints(std::vector<std::vector<double>>& container);

// return data's vector length
int CSV2Data(std::string csv_location, std::string  reading_col_start_title, std::string reading_col_end_title, std::vector<std::vector<double>> &data_vec);

void WaypointRearrange (std::vector<std::vector<double>> &data_vec);

void LatLon2Utm(std::vector<std::vector<double>>& wytps);

void SetVelocityProfile(std::vector<std::vector<double>>& container);

#endif // __WAYPOINT_SAVE_H__