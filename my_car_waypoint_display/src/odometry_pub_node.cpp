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

int main (int argc, char** argv) {

    std::ifstream ifstr_pose ("/home/delcoii/catkin_ws/src/my_car_waypoint_display/src/my_car_odometry_pose.txt");
    

    // read pose position x y z, and orientation x y z w
    std::vector<std::vector<double>> wypt_pose; 

    if (!ifstr_pose) {
        ROS_INFO("cannot find pose file!\n");
        return 0;
    }
    
    int data_size = 0;
    // get pose.position data x, y, z
    while (ifstr_pose) {
        std::vector<double> temp(7, 0);

        for (int i = 0; i < 7; i++) {
            ifstr_pose >> temp[i];
        }
        
        wypt_pose.push_back(temp);
        
        //initialize once
        data_size++;
    }

    std::vector<double>prev_pose = wypt_pose[0];

    for (int i = 1; i < wypt_pose.size()-1; i++) {

        std::vector<double> temp = wypt_pose[i];

        // calculate distance of each waypoints
        double distance = sqrt(pow(prev_pose[POSE_X]-temp[POSE_X], 2) + pow(prev_pose[POSE_Y]-temp[POSE_Y], 2));
        std::cout << distance << std::endl;

        // waypoint closer than 0.19m to previous one
        if (distance < WYPT_DIST) {
            wypt_pose.erase(wypt_pose.begin() + i);
            std::cout << "erased!" << std::endl;
        }
        else {  // distance is farther than 0.19m
            prev_pose = wypt_pose[i];
        }

    }


    /*
    std::ifstream ifstr_twist ("/home/delcoii/catkin_ws/src/my_car_waypoint_display/src/my_car_odometry_twist.txt");

    // read twist linear x y z and angular x y z
    std::vector<std::vector<double>> wypt_twist;

    if (!ifstr_twist) {
        ROS_INFO("cannot find twist file!\n");
        return 0;
    }

    // get twist linear xyz and angular xyz
    while (ifstr_twist) {
        std::vector<double> temp(6, 0);

        for (int i = 0; i < 6; i++) {
            ifstr_twist >> temp[i];
        }
        
        wypt_twist.push_back(temp);
    }*/



    ros::init (argc, argv, "my_car_waypoint_publish");
    ros::NodeHandle nh;

    ros::Rate loop_rate_hz(60);

    ros::Publisher wypt_odo_pub = nh.advertise<nav_msgs::Odometry>("cybertruck_waypoint", 100);

    nav_msgs::Odometry odo2display;
    ros::Time now;
    int displayed_odometry_count = 0;

    while (ros::ok()) {

        now = ros::Time::now();

        odo2display.header.frame_id = "map";
        odo2display.header.stamp = now;


        odo2display.pose.pose.position.x = wypt_pose[displayed_odometry_count][POSE_X];
        odo2display.pose.pose.position.y = wypt_pose[displayed_odometry_count][POSE_Y];
        odo2display.pose.pose.position.z = wypt_pose[displayed_odometry_count][POSE_Z];
        

        odo2display.pose.pose.orientation.x = wypt_pose[displayed_odometry_count][ORIENTATION_X];
        odo2display.pose.pose.orientation.y = wypt_pose[displayed_odometry_count][ORIENTATION_Y];
        odo2display.pose.pose.orientation.z = wypt_pose[displayed_odometry_count][ORIENTATION_Z];
        odo2display.pose.pose.orientation.w = wypt_pose[displayed_odometry_count][ORIENTATION_W];


        /*
        odo2display.twist.twist.linear.x = wypt_twist[displayed_odometry_count][0];
        odo2display.twist.twist.linear.y = wypt_twist[displayed_odometry_count][1];
        odo2display.twist.twist.linear.z = wypt_twist[displayed_odometry_count][2];

        odo2display.twist.twist.angular.x = wypt_twist[displayed_odometry_count][3];
        odo2display.twist.twist.angular.y = wypt_twist[displayed_odometry_count][4];
        odo2display.twist.twist.angular.z = wypt_twist[displayed_odometry_count][5];
        */


        displayed_odometry_count++;
        if (displayed_odometry_count >= wypt_pose.size()) {
            
            std::cout <<
                "readed data :\t" << data_size << "\n" << 
                "published odometry : " << displayed_odometry_count <<
            std::endl;
            displayed_odometry_count = 0;
        }
            
        wypt_odo_pub.publish(odo2display);
        // ROS_INFO("published odometry %d \n", displayed_odometry_count);

        loop_rate_hz.sleep();
    }

}