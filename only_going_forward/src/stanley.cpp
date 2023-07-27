// #include <thread>
// #include <chrono>

#include <ros/ros.h>
#include <tf/tf.h>

#include "only_going_forward/pid.h"
#include "only_going_forward/sub_pub_class.h"
#include "only_going_forward/waypoint_save.h"
#include "only_going_forward/calculating_tools.h"

#include "only_going_forward/parameters.h"





int main(int argc, char** argv) {

    ros::init (argc, argv, "only_going_forward");
    ros::Time::init();

    // declare publishing and subscribing data
    MyCarStatus car_data;
    param p;

    ros::Rate loop_rate_hz(p.LOOP_RATE); // initialize dt 1/60s

    
    // read csv data to column position x ~ orientation w
    std::vector<std::vector<double>> waypoints;
    std::string wypt_csv_loc = "/home/delcoii/waypoint/callback_data.csv";
    std::string wypt_start_col = "field.front_wheel_pose.position.x";
    std::string wypt_end_col = "field.front_wheel_pose.orientation.w";

    // read column data
    std::vector<std::vector<double>> speed_vec;
    std::string speed_csv_loc = "/home/delcoii/waypoint/callback_data.csv";
    std::string speed_start_col = "field.vehicle_status.velocity";
    std::string speed_end_col = speed_start_col;


    CSV2Data(wypt_csv_loc, wypt_start_col, wypt_end_col, waypoints);
    CSV2Data(speed_csv_loc, speed_start_col, speed_end_col, speed_vec);
    // std::cout << "debug" << std::endl;

    for (int idx = 0; idx < waypoints.size(); idx++) {
        waypoints[idx].push_back(speed_vec[idx][0]);
    }

    WaypointRearrange(waypoints);

    // std::chrono::seconds wait_duration(3);
    // std::this_thread::sleep_for(wait_duration);

    // values for PID control
    PID longitudinal_pid = PID((1./p.LOOP_RATE), p.THROTTLE_MAX, p.THROTTLE_MIN, p.P_GAIN, p.D_GAIN, p.I_GAIN);
    double pid_output;


    // values for stanley method
    int target_wypt_idx = 0;
    double wypt_car_dist_m;
    double psi_angle_deg;
    double car_wypt_diff_deg;
    double steering_deg;
    double steering_val;

    // values for calculating waypoint following error
    double min_dist_m;
    double wypt_following_error_avg_m;

    while (ros::ok()) {
        
        tf::Transform wypt_tf;
        wypt_tf.setOrigin (tf::Vector3(
            waypoints[target_wypt_idx][POSITION_X],
            waypoints[target_wypt_idx][POSITION_Y],
            waypoints[target_wypt_idx][POSITION_Z]
        ));
        wypt_tf.setRotation (tf::Quaternion(
            waypoints[target_wypt_idx][ORIENTATION_X],
            waypoints[target_wypt_idx][ORIENTATION_Y],
            waypoints[target_wypt_idx][ORIENTATION_Z],
            waypoints[target_wypt_idx][ORIENTATION_W]
        ));

        tf::Transform front_wheel_tf;
        front_wheel_tf.setOrigin (tf::Vector3(
            car_data.car_status.front_wheel_pose.position.x,
            car_data.car_status.front_wheel_pose.position.y,
            car_data.car_status.front_wheel_pose.position.z
        ));
        front_wheel_tf.setRotation (tf::Quaternion(
            car_data.car_status.front_wheel_pose.orientation.x,
            car_data.car_status.front_wheel_pose.orientation.y,
            car_data.car_status.front_wheel_pose.orientation.z,
            car_data.car_status.front_wheel_pose.orientation.w
        ));

        tf::Transform car_wypt_error_tf = front_wheel_tf.inverseTimes(wypt_tf);

        wypt_car_dist_m = sqrt( pow(car_wypt_error_tf.getOrigin().x(), 2) + pow(car_wypt_error_tf.getOrigin().y(), 2));


        // pid_output = longitudinal_pid.calculate(p.TARGET_VELOCITY_MS, car_data.car_status.vehicle_status.velocity);
        pid_output = longitudinal_pid.calculate(waypoints[target_wypt_idx][SPEED_MS], car_data.car_status.vehicle_status.velocity);


        // calculating psi
        double yaw_error_rad = tf::getYaw(car_wypt_error_tf.getRotation());
        psi_angle_deg = rad2deg(yaw_error_rad);

        
        // calculating arctangent term
        // if waypoint is in right side of my car
        if (car_wypt_error_tf.getOrigin().y() < 0.) {
            wypt_car_dist_m *= (-1.);
        }
        car_wypt_diff_deg = rad2deg(atan(deg2rad(p.STANLEY_CONSTANT * (wypt_car_dist_m / (car_data.car_status.vehicle_status.velocity + p.MIN_ARCTAN_NUMERATOR)))));

        
        // set steering angle
        // supposing cybertruck's steering angle -35 ~ 35 deg
        steering_deg = psi_angle_deg + car_wypt_diff_deg;
        steering_deg = cut(steering_deg, p.CYBERTRUCK_MAX_STEER_DEG, p.CYBERTRUCK_MIN_STEER_DEG);
        steering_val = map(steering_deg, p.CYBERTRUCK_MAX_STEER_DEG, p.CYBERTRUCK_MIN_STEER_DEG, -1.0, 1.0);
        



        // calculating following error
        /*
         * calculating minimum distance of waypoint from front wheel
         * for loop target waypoint index ~ target waypoint index-20
        */ 
        min_dist_m = 9999.;
        for (   int idx = target_wypt_idx;
                idx > target_wypt_idx-50;
                idx--) {
            if (idx <= 3) {
                min_dist_m = 0.;
                break;
            }

            double temp_dist_m = sqrt(
                pow(car_data.car_status.front_wheel_pose.position.x - 
                    waypoints[idx][POSITION_X], 2) +
                pow(car_data.car_status.front_wheel_pose.position.y - 
                    waypoints[idx][POSITION_Y], 2)
            );
            // std::cout << temp_dist_m << std::endl;

            if (min_dist_m > temp_dist_m) {
                min_dist_m = temp_dist_m;
                // std::cout << target_wypt_idx-idx << std::endl;
            }
            
        }

        wypt_following_error_avg_m = avg_filter(min_dist_m);
        // std::cout << wypt_following_error_avg_m << std::endl;


        std::cout << 
            "waypoint idx : " << target_wypt_idx << "\n" << 
            "car to waypoint distance(m) : " << wypt_car_dist_m << "\n" <<
            "psi (deg) : " << psi_angle_deg << "\n" <<
            "target speed (m/s) : " << waypoints[target_wypt_idx][SPEED_MS] << "\n\n" <<

            "minimum distance to waypoint (m) : " << min_dist_m << "\n" << 
            "=========================================" << "\n" <<
        std::endl;

        
        
        if (target_wypt_idx == waypoints.size()-1) {
            car_data.InitPubMsg(ros::Time::now(), 0., 0., 1.);
            car_data.pub_data();
            std::cout << "waypoint following error average(m) : " << wypt_following_error_avg_m << std::endl;
            return 0;
        }
        car_data.InitPubMsg(ros::Time::now(), pid_output, 0., 0.);
        car_data.pub_data();



        // update target waypoint
        if (abs(wypt_car_dist_m) <= p.LOOKAHEAD_DIST_M) {
            target_wypt_idx++;
        }
    
        loop_rate_hz.sleep();
        ros::spinOnce();
    }

    return 0;
}




