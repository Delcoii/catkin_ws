#include <ros/ros.h>

#include "my_car_stanley_method/pid.h"
#include "my_car_stanley_method/sub_pub_class.h"
#include "my_car_stanley_method/waypoint_save.h"


#define LOOP_RATE           60.

#define THROTTLE_MAX        1.0
#define THROTTLE_MIN        0.0

#define P_GAIN              3.0
#define I_GAIN              0.0
#define D_GAIN              0.0

#define TARGET_VELOCITY     10.




int main(int argc, char** argv) {

    ros::init (argc, argv, "my_car_stanley_pid");
    ros::Time::init();
    ros::Rate loop_rate_hz(LOOP_RATE); // initialize dt 1/60s


    std::vector<std::vector<double>> waypoints;
    SaveWaypoint(waypoints);


    // declare publishing and subscribing data
    MyCarStatus car_data;


    PID longitudinal_pid = PID((1./LOOP_RATE), THROTTLE_MAX, THROTTLE_MIN, P_GAIN, I_GAIN, D_GAIN);
    double pid_output;

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
        pid_output = longitudinal_pid.calculate(TARGET_VELOCITY, car_data.velocity_ms);


        // init stamp, throttle, steer
        car_data.InitPubMsg(ros::Time::now(), pid_output, 0);

        car_data.print_val();
        car_data.pub_data();


        loop_rate_hz.sleep();
        ros::spinOnce();
    }

    return 0;
}