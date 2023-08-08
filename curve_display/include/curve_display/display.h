/* Author : Seonghoon Park
 * This class fit curves by using pseudo inverse matrix
 * 
 * if curve y is
 * y = a3x^3 + a2x^2 + a1x + a0
 * y = xA (matrix & vector expression)
 * 
 * so we can predict matrix A by
 * A = x^+ y
 */
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <armadillo>

#define MATRIX_ROW          100
#define MATRIX_COL          4

#define LD                  4.

#define MARKER_ARR_SIZE     100
#define MARKER_DX           0.3     // delta x for dx
#define CURVE_START_IDX     -10     // dx*start_idx is first curve input 
#define CUBE_SIZE           0.1


class CurveDisplayer {

    ros::NodeHandle nh;
    ros::Publisher curve_vis_pub;
    ros::Subscriber front_wheel_sub;
    ros::Subscriber rear_wheel_sub;

    visualization_msgs::MarkerArray curve_vis;

    geometry_msgs::PoseStamped fr_pose;
    geometry_msgs::PoseStamped rr_pose;
    bool msg_received;

    std::vector<std::vector<double>> x_mat;     // assuming col size 4
    std::vector<double> y_vec;                  // assuming vec size 4

    arma::mat x_matrix;
    arma::mat y_vector;
    arma::mat A_matrix;     // what I want to know for curve fitting

    double wypt_car_dist_m;
    
public:
    
    CurveDisplayer();

    void GetFrontWheelPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void GetRearWheelPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    bool SetMatrixCols(std::vector<std::vector<double>> vec,
        int target);   // set 10 waypoints to mat     

    void GetMatA();
    
    void GetCarWyptDist(double _wypt_x, double _wypt_y);
    bool CheckNextTarget(int& idx);

    void SetMarker();
    void Visualize();   // publish marker array

    void PrintInfo();

    // void PublishCurve();

};


