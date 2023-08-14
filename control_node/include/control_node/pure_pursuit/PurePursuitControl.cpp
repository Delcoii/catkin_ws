#include "control_node/pure_pursuit/PurePursuitControl.h"


PurePursuitControl::PurePursuitControl() {

    vehicle_pos_x = 0.;
    vehicle_pos_y = 0.;
    vehicle_pos_yaw_rad = 0.;

    wypt_pos_x = 0.;
    wypt_pos_y = 0.;
    wypt_car_dist_m = 99999.;    // any big num.. 
    wypt_yaw_rad = 0.;

    alpha_deg = 0.;
    theta_deg = 0.;

    target_wypt_idx = 0;
    steering_val = 0;
}


void PurePursuitControl::GetAllWaypoints(std::vector<std::vector<double>> wypts) {
    waypoints = wypts;
}

// find target point & calculate distance
void PurePursuitControl::FindTargetPoint(geometry_msgs::PoseStamped _rr_pose) {

    wypt_car_dist_m = 99999.;         // just a big number
    double temp_dist;
    int search_start = target_wypt_idx + SEARCH_IDX_RANGE;
    int search_end = target_wypt_idx;
    int max_idx = waypoints.size()-1;
    for (int idx = search_start; idx > search_end; idx--) {
        if (idx < 0) {
            break;
        }
        else if (idx > max_idx) {
            continue;
        }
        // init waypoint by tf
        tf::Transform wypt_tf;

        // utm projection index x:[0], y:[1] 
        wypt_tf.setOrigin(tf::Vector3(waypoints[idx][0], waypoints[idx][1], 0.));     
        wypt_tf.setRotation(tf::Quaternion(0., 0., 0., 1.));
        
        // init front wheel by tf
        tf::Transform rr_tf;
        rr_tf.setOrigin(tf::Vector3(
            _rr_pose.pose.position.x,
            _rr_pose.pose.position.y,
            _rr_pose.pose.position.z
        ));
        rr_tf.setRotation(tf::Quaternion(
            _rr_pose.pose.orientation.x, _rr_pose.pose.orientation.y,
            _rr_pose.pose.orientation.z, _rr_pose.pose.orientation.w
        ));

        wypt_tf = rr_tf.inverseTimes(wypt_tf);  // transform wypt_tf
        
        double wypt_x_from_fr = wypt_tf.getOrigin().x();
        double wypt_y_from_fr = wypt_tf.getOrigin().y();

        temp_dist = sqrt( pow(wypt_x_from_fr, 2) + pow(wypt_y_from_fr, 2) );

        if (temp_dist < LOOKAHEAD_DIST) {
            wypt_car_dist_m = temp_dist;
            target_wypt_idx = idx;

            break;
        }
    }

    wypt_pos_x = waypoints[target_wypt_idx][0];
    wypt_pos_y = waypoints[target_wypt_idx][1];
}



double PurePursuitControl::SetSteer(geometry_msgs::PoseStamped _rr_pose) {

    FindTargetPoint(_rr_pose);
    GetRearPos(_rr_pose);
    CalcWyptDist();
    CalcAlpha();
    CalcTheta();

    steering_val = CutMinMax(theta_deg, (-1.)*MAX_STEERING_DEG, MAX_STEERING_DEG);
    steering_val = map(steering_val, (-1.)*MAX_STEERING_DEG, MAX_STEERING_DEG, -1., 1.);

    return steering_val;
}


void PurePursuitControl::GetRearPos(geometry_msgs::PoseStamped _rr_pose) {
    
    vehicle_pos_x = _rr_pose.pose.position.x;
    vehicle_pos_y = _rr_pose.pose.position.y;

    tf::Quaternion quat(
        _rr_pose.pose.orientation.x,
        _rr_pose.pose.orientation.y,
        _rr_pose.pose.orientation.z,
        _rr_pose.pose.orientation.w
    );
    
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    vehicle_pos_yaw_rad = yaw;
}


void PurePursuitControl::CalcWyptDist () {
    
    wypt_car_dist_m = sqrt(
        pow((vehicle_pos_x-wypt_pos_x), 2) +
        pow((vehicle_pos_y-wypt_pos_y), 2)
    );
}

void PurePursuitControl::CalcAlpha () {
    wypt_yaw_rad = atan2(wypt_pos_y - vehicle_pos_y, wypt_pos_x - vehicle_pos_x);
    // wypt_yaw_rad = atan2(vehicle_pos_y - wypt_pos_y, vehicle_pos_x - wypt_pos_x);

    alpha_deg = rad2deg(vehicle_pos_yaw_rad) - rad2deg(wypt_yaw_rad);
    // std::cout << rad2deg(waypoint_yaw);
}

void PurePursuitControl::CalcTheta() {
    // double ld;
    // if (alpha_deg < 0.)
    //     ld = wypt_car_dist_m * -1.;
    // else
    //     ld = wypt_car_dist_m;

    theta_deg = rad2deg(
        atan((2.*WHEEL_BASE_M*sin(deg2rad(alpha_deg))) / wypt_car_dist_m)
    );

    // std::cout << 
    //     "wb : " << WHEEL_BASE_M << "\n" <<
    //     "sin term : " << sin(deg2rad(alpha_deg)) << "\n" <<
    //     (2. * WHEEL_BASE_M * sin(deg2rad(alpha_deg))) << "\n" <<
    //     "atan inside : " << (2.*WHEEL_BASE_M*sin(deg2rad(alpha_deg))) / wypt_car_dist_m << "\n" <<
    // std::endl;
}


bool PurePursuitControl::ArrivedLKDist () {
    if (wypt_car_dist_m < LOOKAHEAD_DIST)
        return true;
    else
        return false;
}


double PurePursuitControl::steer_val() {
    return steering_val;
}


int PurePursuitControl::target_idx() {
    return target_wypt_idx;
}


void PurePursuitControl::PrintValue() {
    std::cout << 
        "vehicle rear pos x : " << vehicle_pos_x << "\n" <<
        "vehicle rear pos y : " << vehicle_pos_y << "\n\n" <<
        
        "waypoint distance (m) : " << wypt_car_dist_m << "\n" << 
        "waypoint yaw(deg) : " << rad2deg(wypt_yaw_rad) << "\n" <<
        "vehicle pos yaw(deg) : " << rad2deg(vehicle_pos_yaw_rad) << "\n" <<
        "alpha(deg) : " << alpha_deg << "\n" <<
        "theta(deg) : " << theta_deg << "\n" <<
    std::endl;
}   



