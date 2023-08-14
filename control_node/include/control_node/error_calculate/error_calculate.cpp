#include "control_node/error_calculate/error_calculate.h"

FollowingError::FollowingError() {
    error_pub = nh.advertise<std_msgs::Float64>("cross_track_error", 100);
    window = std::vector<double> (WINDOW_SIZE, 0);

    average = 0;
    sample_count = 0;
}

void FollowingError::GetCrossTrackError(double dist_m) {
    car_waypoint_dist_m = dist_m;
}

void FollowingError::PutInWindow() {
    window.erase(window.begin());
    window.push_back(car_waypoint_dist_m);
}


double FollowingError::FilteredValue(double dist_m) {

    GetCrossTrackError(dist_m);
    PutInWindow();

    double sum = 0;
    for (int idx = 0; idx < WINDOW_SIZE; idx++) {
        sum += window[idx];
    }
    cross_track_error_m.data = sum / (double)(WINDOW_SIZE);


    sample_count++;

    // calculating alpha = k-1 / k
    double alpha = (double)(sample_count-1) / (double)sample_count;
    // calculating average
    average = alpha * average + (1.-alpha) * cross_track_error_m.data;

    return cross_track_error_m.data;
}

double FollowingError::err_avg() {
    return average;
}

void FollowingError::PubCrossTrackError() {
    error_pub.publish(cross_track_error_m);
}

