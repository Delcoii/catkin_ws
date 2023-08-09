#include "curve_display/display.h"

CurveDisplayer::CurveDisplayer() : 
    x_matrix(MATRIX_ROW, MATRIX_COL),
    y_vector(MATRIX_ROW, 1),
    A_matrix(4, 1)
{
    msg_received = false;

    curve_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/curve_to_follow", 100);
    front_wheel_sub = nh.subscribe("/front_wheel_pose", 100, &CurveDisplayer::GetFrontWheelPose, this);
    rear_wheel_sub = nh.subscribe("/rear_wheel_pose", 100, &CurveDisplayer::GetRearWheelPose, this);


    curve_vis.markers.resize(MARKER_ARR_SIZE);
    for (int i = 0; i < MARKER_ARR_SIZE; i++) {
        curve_vis.markers[i].header.frame_id = "front_wheel_center";
        curve_vis.markers[i].ns = "fitting_curve";
        curve_vis.markers[i].id = i;
        curve_vis.markers[i].type = visualization_msgs::Marker::CUBE;
        curve_vis.markers[i].action = visualization_msgs::Marker::ADD;

        curve_vis.markers[i].scale.x = CUBE_SIZE;
        curve_vis.markers[i].scale.y = CUBE_SIZE;
        curve_vis.markers[i].scale.z = CUBE_SIZE;

        // green
        curve_vis.markers[i].color.r = 1.0f;
        curve_vis.markers[i].color.g = 0.8f;
        curve_vis.markers[i].color.b = 0.5f;
        curve_vis.markers[i].color.a = 1.0f;

        curve_vis.markers[i].lifetime = ros::Duration();
    }

}

void CurveDisplayer::GetFrontWheelPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    fr_pose.pose = msg->pose;
    msg_received = true;
}
void CurveDisplayer::GetRearWheelPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    rr_pose.pose = msg->pose;
    msg_received = true;
}

bool CurveDisplayer::SetMatrixCols(std::vector<std::vector<double>> vec, int target) {
    if (vec.empty() == true) {
        std::cout << "cannot set matrix cols cause container empty" << std::endl;
        return false;
    }
    else {
        x_mat.clear();
        y_vec.clear();

        if (msg_received == false) {
            return false;
        }

        int fitting_start = target-20;
        int fitting_end = target+MATRIX_ROW;
        for (int idx = fitting_start; idx < fitting_end; idx++) {
            
            if (idx < 0) {
                continue;
            }
            tf::Transform front_wheel_tf;
            tf::Transform wypt_tf;
            wypt_tf.setOrigin(tf::Vector3(
                vec[idx][0],
                vec[idx][1],
                0.
            ));
            wypt_tf.setRotation(tf::Quaternion(
                0., 0., 0., 1.
            ));

            front_wheel_tf.setOrigin(tf::Vector3(
                fr_pose.pose.position.x,
                fr_pose.pose.position.y,
                fr_pose.pose.position.z
            ));
            front_wheel_tf.setRotation(tf::Quaternion(
                fr_pose.pose.orientation.x,
                fr_pose.pose.orientation.y,
                fr_pose.pose.orientation.z,
                fr_pose.pose.orientation.w
            ));

            wypt_tf = front_wheel_tf.inverseTimes(wypt_tf);

            /* for debugging
            std::cout <<
                "fr_pose\n" << 
                "x" << fr_pose.pose.position.x << "\n" <<
                "y" << fr_pose.pose.position.y << "\n" <<
                "z" << fr_pose.pose.position.z << "\n" <<
            std::endl;
            */
            
            double x3 = pow(wypt_tf.getOrigin().x(), 3);
            double x2 = pow(wypt_tf.getOrigin().x(), 2);
            double x1 = wypt_tf.getOrigin().x();
            double x0 = 1.;
            
            std::vector<double> x_column;
            x_column.push_back(x3);
            x_column.push_back(x2);
            x_column.push_back(x1);
            x_column.push_back(x0);

            x_mat.push_back(x_column);
            y_vec.push_back(wypt_tf.getOrigin().y());
        }
        return true;
    }
}

/*
 * Set Columns for x matrix and y vector
 * assuming waypoint input is (latitude, longitude)
 
void CurveDisplayer::UpdateMatrixCols(std::vector<double> wypt_input) {
    if (wypt_input.size() != 2) {
        std::cout << "input column size is not 2!!!" << std::endl;
        return;
    }
    
    // transform waypoint based to front wheel center
    tf::Transform front_wheel_tf;
    tf::Transform wypt_tf;
    wypt_tf.setOrigin(tf::Vector3(
        wypt_input[0],  // latitude to utm projection value
        wypt_input[1],  // longitude to utm projection value
        0.
    ));
    wypt_tf.setRotation(tf::Quaternion(
        0., 0., 0., 1.
    ));

    front_wheel_tf.setOrigin(tf::Vector3(
        fr_pose.pose.position.x,
        fr_pose.pose.position.y,
        fr_pose.pose.position.z
    ));
    front_wheel_tf.setRotation(tf::Quaternion(
        fr_pose.pose.orientation.x,
        fr_pose.pose.orientation.y,
        fr_pose.pose.orientation.z,
        fr_pose.pose.orientation.w
    ));
    wypt_tf = front_wheel_tf.inverseTimes(wypt_tf);

    x_mat.erase(x_mat.begin());
    y_vec.erase(y_vec.begin());
    
    
    std::vector<double> input_col;
    double x3, x2, x1, x0;
    x3 = pow(wypt_tf.getOrigin().x(), 3);
    x2 = pow(wypt_tf.getOrigin().x(), 2);
    x1 = wypt_tf.getOrigin().x();
    x0 = 1.;
    input_col.push_back(x3);
    input_col.push_back(x2);
    input_col.push_back(x1);
    input_col.push_back(x0);
    
    x_mat.push_back(input_col);
    y_vec.push_back(wypt_tf.getOrigin().y());

}
*/


void CurveDisplayer::GetMatA() {
    for (int row = 0; row < MATRIX_ROW; row++) {
        for (int col = 0; col < MATRIX_COL; col++) {
            x_matrix(row, col) = x_mat[row][col];
        }

        y_vector(row, 0) = y_vec[row];
    }
    arma::mat temp = arma::pinv(x_matrix);
    A_matrix = temp * y_vector;
}


void CurveDisplayer::SetMarker() {
    for (int i = 0 ; i < MARKER_ARR_SIZE; i++) {
        double input_x = double(i+CURVE_START_IDX) * MARKER_DX;
        
        double a3x3 = A_matrix(0, 0) * pow(input_x, 3);
        double a2x2 = A_matrix(1, 0) * pow(input_x, 2);
        double a1x1 = A_matrix(2, 0) * input_x;
        double a0 = A_matrix(3, 0);

        curve_vis.markers[i].header.stamp = ros::Time::now();
        curve_vis.markers[i].pose.position.x = input_x;
        curve_vis.markers[i].pose.position.y = a3x3 + a2x2 + a1x1 + a0;
        curve_vis.markers[i].pose.position.z = 0.;
        curve_vis.markers[i].pose.orientation.x = 0.0;
        curve_vis.markers[i].pose.orientation.y = 0.0;
        curve_vis.markers[i].pose.orientation.z = 0.0;
        curve_vis.markers[i].pose.orientation.w = 0.0;
    }
}


void CurveDisplayer::Visualize() {
    curve_vis_pub.publish(curve_vis);
}


void CurveDisplayer::GetCarWyptDist(double _wypt_x, double _wypt_y) {
    wypt_car_dist_m = sqrt(
        pow(fr_pose.pose.position.x-_wypt_x, 2) +
        pow(fr_pose.pose.position.y-_wypt_y, 2)
    );
}


bool CurveDisplayer::CheckNextTarget(int& idx) {
    if (wypt_car_dist_m < LD) {
        idx++;
        return true;
    }
    else {
        return false;
    }
}




void CurveDisplayer::PrintInfo() {
    /* for debugging 
    for (int i = 0; i < MATRIX_ROW; i++) {
        for (int j = 0; j < MATRIX_COL; j++) {
            std::cout << x_mat[i][j] << "  ";
        }
        std::cout << "\n";
    } */
    

    // std::cout << "x mat : \n";
    // x_matrix.print();
    // std::cout << "y vec : \n";
    // y_vector.print();
    std::cout << "A vector : \n";
    A_matrix.print();
    std::cout << "waypoint distance : " << wypt_car_dist_m << std::endl;
}