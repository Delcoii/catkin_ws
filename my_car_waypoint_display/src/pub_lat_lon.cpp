#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_projection/UTM.h"


#define POSITION_X              0
#define POSITION_Y              1
#define POSITION_Z              2
#define ORIENTATION_X           3
#define ORIENTATION_Y           4
#define ORIENTATION_Z           5
#define ORIENTATION_W           6

#define WYPT_DIST_M             0.6

#define REF_LATITUDE            0.
#define REF_LONGITUDE           0.

int CSV2Data(std::string csv_location, std::string  reading_col_start_title, std::string reading_col_end_title, std::vector<std::vector<double>> &data_vec);

void WaypointRearrange (std::vector<std::vector<double>> &data_vec);



int main (int argc, char** argv) {


    std::vector<std::vector<double>> waypoints;
    std::string wypt_csv_loc = "/home/delcoii/bagfiles/reference_bag/odo.csv";
    std::string wypt_start_col = "field.pose.pose.position.x";
    std::string wypt_end_col = "field.pose.pose.position.y";


    
    // using car odometry because front wheel orientation update delayed

    
    CSV2Data(wypt_csv_loc, wypt_start_col, wypt_end_col, waypoints);
    // WaypointRearrange(waypoints);

    /*
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
    */


    ros::init (argc, argv, "my_car_waypoint_publish");
    ros::NodeHandle nh;

    ros::Rate loop_rate_hz(60);

    ros::Publisher wypt_odo_pub = nh.advertise<nav_msgs::Odometry>("waypoints", 100);

    nav_msgs::Odometry odo2display;
    ros::Time now;
    int displayed_odometry_count = 0;

    while (ros::ok()) {

        now = ros::Time::now();

        odo2display.header.frame_id = "map";
        odo2display.header.stamp = now;


        odo2display.pose.pose.position.x = waypoints[displayed_odometry_count][POSITION_X];
        odo2display.pose.pose.position.y = waypoints[displayed_odometry_count][POSITION_Y];


        odo2display.pose.pose.position.z = 0.;
        

        odo2display.pose.pose.orientation.x = 0.;
        odo2display.pose.pose.orientation.y = 0.;
        odo2display.pose.pose.orientation.z = 0.;
        odo2display.pose.pose.orientation.w = 1.;


        displayed_odometry_count++;
        wypt_odo_pub.publish(odo2display);
        if (displayed_odometry_count >= waypoints.size()) {
            
            std::cout <<
                "readed data :\t" << waypoints.size() << "\n" << 
                "published odometry : " << displayed_odometry_count <<
            std::endl;
            // displayed_odometry_count = 0;
        }
            
        
        // ROS_INFO("published odometry %d \n", displayed_odometry_count);
        if (displayed_odometry_count == waypoints.size()-1)
            return 0;

        loop_rate_hz.sleep();
    }

}


int CSV2Data(std::string csv_location, std::string  reading_col_start_title, std::string reading_col_end_title, std::vector<std::vector<double>> &data_vec) {
    
    std::ifstream data_stream (csv_location);

    if (!data_stream.is_open()) {
        ROS_INFO("cannot find data file!\n");
        return -1;
    }

    std::string input_line;
    std::string temp_str;
    int reading_col_start = 0;
    int reading_col_end = 0;

    // getting first line to set range of column to read
    std::getline(data_stream, input_line);
    std::stringstream temp_ss(input_line);


    // set starting column to read
    while (std::getline(temp_ss, temp_str, ',')) {

        if (temp_str == reading_col_start_title) {
            reading_col_end = reading_col_start;
            break;
        }
        else {
            reading_col_start++;
        }        
    }

    // get end column to read
    // if start & end is same, break immediately
    do {
        
        if (temp_str == reading_col_end_title) {
            break;
        }
        else {
            reading_col_end++;
        }

    }while (std::getline(temp_ss, temp_str, ','));


    int data_width = reading_col_end - reading_col_start + 1;
    std::cout << "reading column width : " << data_width << std::endl;

    // save data
    // getline's default delim is  '\n'
    while (std::getline(data_stream, input_line)) {

        std::vector<double> temp_data(data_width, 0);
        
        std::stringstream temp_ss(input_line);
        std::string temp_str;

        // ignoring columns before starting column
        for (int count = 0; count < reading_col_start; count++) {
            std::getline(temp_ss, temp_str, ',');
        }

        // save data in vec
        for (int count = 0; count < data_width; count++) {
            std::getline(temp_ss, temp_str, ',');
            temp_data[count] = std::stod(temp_str);
        }

        // ignoring other columns
        std::getline(temp_ss, temp_str);

        data_vec.push_back(temp_data);
    }
    data_stream.close();

    return data_width;
}


void WaypointRearrange (std::vector<std::vector<double>> &data_vec) {

    std::vector<double> prev_pose = data_vec[0];

    for (int i = 1; i < data_vec.size()-1; i++) {

        std::vector<double> temp = data_vec[i];

        // calculate distance of each waypoints
        double distance = sqrt(pow(prev_pose[POSITION_X]-temp[POSITION_X], 2) + pow(prev_pose[POSITION_Y]-temp[POSITION_Y], 2));
        std::cout << distance << std::endl;

        // closer than WYPT_DIST to previous one
        if (distance < WYPT_DIST_M) {
            data_vec.erase(data_vec.begin() + i);
            // std::cout << "erased!" << std::endl;
        }
        else {  // distance is farther than 0.19m
            prev_pose = data_vec[i];
        }

    }
    std::cout << "waypoint size : " << data_vec.size();
}