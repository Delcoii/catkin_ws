#include <cmath>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
// #include <geometry_msgs/TransformStamped.h>

#include "my_car_stanley_method/pid.h"
#include "my_car_stanley_method/sub_pub_class.h"
#include "my_car_stanley_method/waypoint_save.h"


#define LOOP_RATE                   60.

#define THROTTLE_MAX                1.0
#define THROTTLE_MIN                0.0

#define P_GAIN                      3.0
#define I_GAIN                      0.0
#define D_GAIN                      1.0
#define TARGET_VELOCITY_MS          15.

#define LOOKAHEAD_DIST_M            3.
#define LEFT                        1
#define RIGHT                       -1
#define STANLEY_CONSTANT            7.
#define CYBERTRUCK_MAX_STEER_DEG    35.
#define CYBERTRUCK_MIN_STEER_DEG    -35.




double rad2deg(double rad);
double deg2rad(double deg);
double cut(double value, double max, double min);
double map(double x, double in_min, double in_max, double out_min, double out_max);
int WyptLeftOrRight(std::vector<double> waypoint, nav_msgs::Odometry car_odometry);

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
    PID longitudinal_pid = PID((1./LOOP_RATE), THROTTLE_MAX, THROTTLE_MIN, P_GAIN, D_GAIN, I_GAIN);
    double pid_output;

    // values for stanley method
    int target_wypt_idx = 0;
    double wypt_car_dist_m;
    double psi_angle_deg;
    double car_wypt_diff_deg;
    double steering_deg;
    double steering_val;

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


        // calculating psi, + degree for CCW
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







        // calculating arctangent term
        if (WyptLeftOrRight(waypoints[target_wypt_idx], car_data.car_odometry) == RIGHT) {
            wypt_car_dist_m *= (-1.);
        }
        car_wypt_diff_deg = rad2deg(atan(deg2rad(STANLEY_CONSTANT * (wypt_car_dist_m / car_data.velocity_ms))));


        // set steering angle
        // supposing cybertruck's steering angle -35 ~ 35 deg
        steering_deg = psi_angle_deg + car_wypt_diff_deg;
        steering_deg = cut(steering_deg, CYBERTRUCK_MAX_STEER_DEG, CYBERTRUCK_MIN_STEER_DEG);
        steering_val = map(steering_deg, CYBERTRUCK_MAX_STEER_DEG, CYBERTRUCK_MIN_STEER_DEG, -1.0, 1.0);
        


        std::cout << 
            "calculated data : \n" << 
            "target waypoint : " << target_wypt_idx << "\n" <<
            "    x : " << waypoints[target_wypt_idx][POSE_X] << "\n" <<
            "    y : " << waypoints[target_wypt_idx][POSE_Y] << "\n" <<
            "    z : " << waypoints[target_wypt_idx][POSE_Z] << "\n" <<
            "distance : " << wypt_car_dist_m << "\n\n" <<

            "psi_angle_deg\t\t: " << psi_angle_deg << "\n" <<
            "car_wypt_diff_deg\t: " << car_wypt_diff_deg << "\n" << 
            "steering_deg\t\t: " << steering_deg << "\n" <<
            "steering_val\t\t: " << steering_deg << "\n" <<
        std::endl;
        

        // init stamp, throttle, steer
        car_data.InitPubMsg(ros::Time::now(), pid_output, steering_val);

        car_data.print_val();
        car_data.pub_data();

        if (target_wypt_idx == 775)
            return 0;

        loop_rate_hz.sleep();
        ros::spinOnce();
    }

    return 0;
}


double rad2deg(double rad) {
    return rad*180./M_PI;
}

double deg2rad(double deg) {
    return deg*(M_PI/180.);
}

double cut(double value, double max, double min) {
    if (value >= max)
        return max;
    else if (value <= min)
        return min;
    else
        return value;
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int WyptLeftOrRight(std::vector<double> waypoint, nav_msgs::Odometry car_odometry) {

    double vehicle_x = car_odometry.pose.pose.position.x;
    double vehicle_y = car_odometry.pose.pose.position.y;
    double vehicle_orientation = tf::getYaw(car_odometry.pose.pose.orientation);

    double target_x = waypoint[POSE_X];
    double target_y = waypoint[POSE_Y];

    double vehicle_direction_x = cos(vehicle_orientation);
    double vehicle_direction_y = sin(vehicle_orientation);

    double relative_vector_x = target_x - vehicle_x;
    double relative_vector_y = target_y - vehicle_y;

    double cross_product = vehicle_direction_x * relative_vector_y - vehicle_direction_y * relative_vector_x;
    
    if (cross_product > 0.) {
        return LEFT;  
    } else if (cross_product <= 0.) {
        return RIGHT;
    } else {
        return 0;
    }

}