#include "control_node/stanley/stanley.h"

StanleyControl::StanleyControl() {
    target_wypt_idx = 0;

}

void StanleyControl::GetAllWaypoints(std::vector<std::vector<double>> wypts) {
    waypoints = wypts;
}

// set waypoint, its distance, and psi
void StanleyControl::FindTargetPoint(geometry_msgs::PoseStamped fr_pose) {

    car_wypt_dist_m = 99999.;         // just a big number
    double temp_dist;
    int search_start = target_wypt_idx - SEARCH_IDX_RANGE;
    int search_end = target_wypt_idx + SEARCH_IDX_RANGE;
    int max_idx = waypoints.size();
    for (int idx = search_start; idx < search_end; idx++) {
        if (idx < 0) {
            continue;
        }
        else if (idx > max_idx -1) {
            break;
        }
        // init waypoint by tf
        tf::Transform wypt_tf;

        // utm projection index x:[0], y:[1] 
        wypt_tf.setOrigin(tf::Vector3(waypoints[idx][0], waypoints[idx][1], 0.));     
        wypt_tf.setRotation(tf::Quaternion(0., 0., 0., 1.));
        
        // init front wheel by tf
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

        wypt_tf = front_wheel_tf.inverseTimes(wypt_tf);     // transform wypt_tf, standard to front_wheel_tf
        

        double wypt_x_from_fr = wypt_tf.getOrigin().x();
        double wypt_y_from_fr = wypt_tf.getOrigin().y();

        temp_dist = sqrt( pow(wypt_x_from_fr, 2) + pow(wypt_y_from_fr, 2) );
        if (temp_dist < car_wypt_dist_m) {
            car_wypt_dist_m = temp_dist;
            target_wypt_idx = idx;
            target_wypt_tf = wypt_tf;
        }
    }
}

void StanleyControl::GetPsi() {
    tf::Transform next_tf;
    next_tf.setOrigin(tf::Vector3(
        waypoints[target_wypt_idx+1][0],
        waypoints[target_wypt_idx+1][1],
        0.
    ));     
    next_tf.setRotation(tf::Quaternion(0., 0., 0., 1.));
    next_wypt_tf = front_wheel_tf.inverseTimes(next_tf);     // transform wypt_tf standard to front_wheel_tf

    psi_deg = rad2deg(atan2(
        next_wypt_tf.getOrigin().y() - target_wypt_tf.getOrigin().y(),
        next_wypt_tf.getOrigin().x() - target_wypt_tf.getOrigin().x()
    ));
}

void StanleyControl::GetArcTanTerm(double _velocity) {
    double direction_include_dist_m;

    /*
    std::cout <<
        "tf y : " << target_wypt_tf.getOrigin().y() << "\n" <<
        "tf x : " << target_wypt_tf.getOrigin().x() << "\n" <<
    std::endl;
    */

    if (target_wypt_tf.getOrigin().y() > 0) {
        direction_include_dist_m = car_wypt_dist_m;
    }
    else {
        direction_include_dist_m = car_wypt_dist_m * (-1.);
    }

    if (_velocity > 0.2) {
        arctan_term_deg = rad2deg(atan(deg2rad(
        ARCTAN_TERM_CONSTANT * direction_include_dist_m /
        (_velocity + ARCTAN_TERM_MIN_VELOCITY))));
    }
    else {
        arctan_term_deg = rad2deg(atan(deg2rad(
        ARCTAN_TERM_CONSTANT * direction_include_dist_m /
        (_velocity + 3.))));
    }
}

/*
double StanleyControl::GetSteeringValue() {
    double steering_angle = psi_deg + arctan_term_deg;
    // double steering_angle = psi_deg;    // for testing
    steering_angle = CutMinMax(steering_angle, MIN_STEERING_DEG, MAX_STEERING_DEG);
    steering_val = map(steering_angle, MIN_STEERING_DEG, MAX_STEERING_DEG, 1.0, -1.0);

    return steering_val;
}*/

double StanleyControl::SetSteer(geometry_msgs::PoseStamped _fr_pose, double _velocity) {
    FindTargetPoint(_fr_pose);
    GetPsi();
    GetArcTanTerm(_velocity);

    double steering_angle = psi_deg + arctan_term_deg;
    // double steering_angle = psi_deg;    // for testing
    steering_angle = CutMinMax(steering_angle, MIN_STEERING_DEG, MAX_STEERING_DEG);
    steering_val = map(steering_angle, MIN_STEERING_DEG, MAX_STEERING_DEG, 1.0, -1.0);

    return steering_val;
}


double StanleyControl::distance_m() {
    return car_wypt_dist_m;
}

double StanleyControl::steer_val() {
    return steering_val;
}

void StanleyControl::PrintValue() {
    std::cout << 
        "target waypoint index : " << target_wypt_idx << "\n" <<
        "waypoint distance (m) : " << car_wypt_dist_m << "\n" <<
        "psi (degree) : " << psi_deg << "\n" <<
        "arctan term (degree) : " << arctan_term_deg << "\n" <<
        "theta (degree) : " << psi_deg + arctan_term_deg << "\n" << 
        "steer : " << steering_val << "\n" <<
    std::endl;
}


int StanleyControl::target_idx() {
    return target_wypt_idx;
}

