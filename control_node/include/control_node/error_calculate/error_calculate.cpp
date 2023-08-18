#include "control_node/error_calculate/error_calculate.h"

FollowingError::FollowingError() {
    error_pub = nh.advertise<control_node::ErrorMsgs>("following_errors", 100);
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

/* CTE : cross track error */
double FollowingError::FilteringCTE(double dist_m) {

    /* init cross_track_error by using moving average filter */
    GetCrossTrackError(dist_m);
    PutInWindow();

    double sum = 0;
    for (int idx = 0; idx < WINDOW_SIZE; idx++) {
        sum += window[idx];
    }
    errors.cross_track_error = sum / (double)(WINDOW_SIZE);
    /*********************************************************/


    /* calculate average of cross track error */
    sample_count++;

    double alpha = (double)(sample_count-1) / (double)sample_count;
    average = alpha * average + (1.-alpha) * errors.cross_track_error;
    /******************************************/

    return errors.cross_track_error;
}

double FollowingError::cte_err_avg() {
    return average;
}

void FollowingError::GetSpeed(double target_speed, double now_speed) {
    errors.target_speed_ms = target_speed;
    errors.current_speed_ms = now_speed;
}


void FollowingError::PubError() {
    error_pub.publish(errors);
}

