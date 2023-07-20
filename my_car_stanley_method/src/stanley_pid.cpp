#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "my_car_stanley_method/pid.h"
#include "my_car_stanley_method/sub_pub_class.h"
#include "my_car_stanley_method/waypoint_save.h"


#define LOOP_RATE               60.

#define THROTTLE_MAX            1.0
#define THROTTLE_MIN            0.0

#define P_GAIN                  3.0
#define I_GAIN                  0.0
#define D_GAIN                  0.0
#define TARGET_VELOCITY_MS      10.

#define LOOKAHEAD_DIST_M        5.


double rad2deg(double rad);

int main(int argc, char** argv) {

    ros::init (argc, argv, "my_car_stanley_pid");
    ros::Time::init();
    ros::Rate loop_rate_hz(LOOP_RATE); // initialize dt 1/60s


    std::vector<std::vector<double>> waypoints;
    SaveWaypoint(waypoints);


    /*
    for (int i = 0; i < waypoints.size(); i++) {
        std::cout << waypoints[i][POSE_X] << std::endl;
    }*/

    // declare publishing and subscribing data
    MyCarStatus car_data;

    // values for PID control
    PID longitudinal_pid = PID((1./LOOP_RATE), THROTTLE_MAX, THROTTLE_MIN, P_GAIN, I_GAIN, D_GAIN);
    double pid_output;

    // values for stanley method
    int target_wypt_idx = 0;
    double psi_angle_deg;
    double wypt_car_dist_m;

    while (ros::ok()) {
        
        
 
        /* Longitudinal Control
         *
         * target speed     : 10m/s
         * measured speed   : velocity subscribed from ego_vehicle/vehicle_status 
         * P GAIN           : 3.0
         * I GAIN           : 0.0
         * D GAIN           : 0.0
         * dt               : 1/60(s)
        */
        pid_output = longitudinal_pid.calculate(TARGET_VELOCITY_MS, car_data.velocity_ms);

        // dist^2 = (x1-x2)^2 + (y1-y2)^2
        wypt_car_dist_m = sqrt(pow((car_data.front_wheel_pos_x-waypoints[target_wypt_idx][POSE_X]), 2) + pow((car_data.front_wheel_pos_y-waypoints[target_wypt_idx][POSE_Y]), 2));

        // update target waypoint
        if (wypt_car_dist_m <= LOOKAHEAD_DIST_M) {
            target_wypt_idx++;
        }

        
        // for conversion, quaternion to euler 
        tf::Quaternion car_quat(
            car_data.car_odometry.pose.pose.orientation.x,
            car_data.car_odometry.pose.pose.orientation.y,
            car_data.car_odometry.pose.pose.orientation.z,
            car_data.car_odometry.pose.pose.orientation.w);
        
        tf::Quaternion wypt_quat(
            waypoints[target_wypt_idx][ORIENTATION_X],
            waypoints[target_wypt_idx][ORIENTATION_Y],
            waypoints[target_wypt_idx][ORIENTATION_Z],
            waypoints[target_wypt_idx][ORIENTATION_W]);
        
        tf::Matrix3x3 car_mat(car_quat);
        tf::Matrix3x3 wypt_mat(wypt_quat);

        double car_roll_rad, car_pitch_rad, car_yaw_rad, wypt_roll_rad, wypt_pitch_rad, wypt_yaw_rad;
        car_mat.getRPY(car_roll_rad, car_pitch_rad, car_yaw_rad);
        wypt_mat.getRPY(wypt_roll_rad, wypt_pitch_rad, wypt_yaw_rad);

        double car_yaw_deg = rad2deg(car_yaw_rad);
        double wypt_yaw_deg = rad2deg(wypt_yaw_rad);
        
        // normalization
        car_yaw_deg += 180.;
        wypt_yaw_deg += 180.;

        double angle_diff_deg = car_yaw_deg - wypt_yaw_deg;
        if (angle_diff_deg >= 180.) {
            psi_angle_deg = (360. - angle_diff_deg);
        } else if (angle_diff_deg >= 0. && angle_diff_deg < 180.) {
            psi_angle_deg = (-1.)*angle_diff_deg;
        } else if (angle_diff_deg >= -180. && angle_diff_deg < 0) {
            psi_angle_deg = (-1.)*angle_diff_deg;
        } else {// -360 ~ 180
            psi_angle_deg = (-1.)*(360. + angle_diff_deg);
        }

        
        std::cout << 
            "calculated data : \n" << 
            "target waypoint : " << target_wypt_idx << "\n" <<
            "    x : " << waypoints[target_wypt_idx][POSE_X] << "\n" <<
            "    y : " << waypoints[target_wypt_idx][POSE_Y] << "\n" <<
            "    z : " << waypoints[target_wypt_idx][POSE_Z] << "\n" <<
            "distance : " << wypt_car_dist_m << "\n\n" <<

            "car yaw(deg)\t\t: " << car_yaw_deg << "\n" <<
            "waypoint yaw(deg)\t: " << wypt_yaw_deg << "\n" <<
            "psi_angle_deg(deg)\t: " << psi_angle_deg << "\n" <<
            "\n\n===========" << 
        std::endl;
        


        // init stamp, throttle, steer
        car_data.InitPubMsg(ros::Time::now(), pid_output, 0);

        car_data.print_val();
        car_data.pub_data();


        loop_rate_hz.sleep();
        ros::spinOnce();
    }

    return 0;
}


double rad2deg(double rad) {
    return rad*180./M_PI;
}