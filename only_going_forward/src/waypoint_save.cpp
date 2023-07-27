#include "only_going_forward/waypoint_save.h"


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

    std::cout << "reading column width : " << data_width << std::endl;
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