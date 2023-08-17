// save latitude, longitude
// executable linked to following_lat_lon.cpp
#include "control_node/waypoint_save/waypoint_save.h"



void GetWaypoints (std::vector<std::vector<double>>& container) {

    std::string csv_loc = "/home/delcoii/waypoints/lane4th.csv";
    std::string start_col = "latitude";
    std::string end_col = "longitude";

    CSV2Data(csv_loc, start_col, end_col, container);

    // WaypointRearrange(container);
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
        // std::cout << distance << std::endl;

        // closer than WYPT_DIST to previous one
        if (distance < WYPT_DIST_M) {
            data_vec.erase(data_vec.begin() + i);
            // std::cout << "erased!" << std::endl;
        }
        else {  // distance is farther than 0.19m
            prev_pose = data_vec[i];
        }

    }
    std::cout << "waypoint size : " << data_vec.size() << std::endl;
}



void LatLon2Utm(std::vector<std::vector<double>>& wypts) {
    // utm projection
    lanelet::Origin origin({REF_LATITUDE, REF_LONGITUDE});
    auto projector = lanelet::projection::UtmProjector(origin);
    for (int idx = 0; idx < wypts.size(); idx++) {
        lanelet::BasicPoint3d utm_coords = projector.forward(
            {wypts[idx][LATITUDE_IDX],
            wypts[idx][LONGITUDE_IDX],
            0}
        );

        wypts[idx][LATITUDE_IDX] = utm_coords.x();
        wypts[idx][LONGITUDE_IDX] = utm_coords.y();
    }
    std::cout << "projection complete, waypoint size : " << wypts.size() << std::endl;
}

void SetVelocityProfile(std::vector<std::vector<double>>& container) {
    
    // MovingAverage speed_mov_avg(SPEED_AVG_WINDOW_SIZE, STRAIGHT_SPEED_MS);
    // MovingAverage kappa_mov_avg(KAPPA_AVG_WINDOW_SIZE, 0.);
    std::vector<double> velocity_container;
    std::vector<double> kappa_container;

    for (int idx = 0; idx < IDX_DIFF; idx++) {
        velocity_container.push_back(STRAIGHT_SPEED_MS);
    }

    /* calculating curvature(kappa) */
    // exclude very first & last index
    for (int idx = IDX_DIFF; idx < container.size()-IDX_DIFF; idx++) {
        arma::vec prev_now(3, arma::fill::zeros);
        arma::vec now_prev(3, arma::fill::zeros);
        arma::vec next_now(3, arma::fill::zeros);
        arma::vec next_prev(3, arma::fill::zeros);

        prev_now = {
            container[idx-IDX_DIFF][LOCAL_X] - container[idx][LOCAL_X],
            container[idx-IDX_DIFF][LOCAL_Y] - container[idx][LOCAL_Y],
            0.
        };
        now_prev = {
            container[idx][LOCAL_X] - container[idx-IDX_DIFF][LOCAL_X],
            container[idx][LOCAL_Y] - container[idx-IDX_DIFF][LOCAL_Y],
            0.
        };        
        next_now = {
            container[idx+IDX_DIFF][LOCAL_X] - container[idx][LOCAL_X],
            container[idx+IDX_DIFF][LOCAL_Y] - container[idx][LOCAL_Y],
            0.
        };        
        next_prev = {
            container[idx+IDX_DIFF][LOCAL_X] - container[idx-IDX_DIFF][LOCAL_X],
            container[idx+IDX_DIFF][LOCAL_Y] - container[idx-IDX_DIFF][LOCAL_Y],
            0.
        };        

        double boonja = 2. * arma::norm(arma::cross(next_now, prev_now), 2);
        double boonmo = arma::norm(next_now, 2) * arma::norm(now_prev, 2) * arma::norm(next_prev, 2);
        double kappa = boonja / boonmo;
        kappa_container.push_back(kappa);
        
        double max_vel_ms = sqrt(MAX_LATERAL_ACCEL_MS2 / (kappa));
        max_vel_ms = CutMinMax(max_vel_ms, 0.0, STRAIGHT_SPEED_MS);
        // max_vel_ms = speed_mov_avg.Filter(max_vel_ms); 

        velocity_container.push_back(max_vel_ms);
        
        std::cout <<
            "\n\nidx : " << idx << "\n" <<
            "boonja : " << boonja << "\n" << 
            "boonmo : " << boonmo << "\n" <<
            "kappa : " << kappa << "\n" <<
            "vel : " << max_vel_ms << "\n" <<
            velocity_container.size() << "\n" <<
        std::endl;   
    }


    // fill start & end index velocity
    for(int idx = IDX_DIFF; idx > 0; idx--) {
        velocity_container[idx-1] = velocity_container[idx];
        velocity_container.push_back(velocity_container[velocity_container.size()-1]);
    }

    int vel_len = velocity_container.size();
    std::cout << "velocity set total size : " << vel_len << std::endl;

    std::vector<double> temp = velocity_container;
    for (int idx = (GAUSSIAN_WINDOW_SIZE/2); idx < vel_len-(GAUSSIAN_WINDOW_SIZE/2); idx++) {
        velocity_container[idx] = GaussianFilter(temp, GAUSSIAN_WINDOW_SIZE, idx);
    }

    for (int idx = 0; idx < container.size(); idx++) {
        container[idx].push_back(velocity_container[idx]);
        container[idx].push_back(kappa_container[idx]);
    }
}