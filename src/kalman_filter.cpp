#include <iostream>
#include <pattern_follower/kalman_filter.h>

KalmanFilter_::KalmanFilter_(const double &init_x, const double &init_y) {
        F = (cv::Mat_<double>(6, 6) << 1, 0, 1, 0, 0.5, 0, 0, 1, 0, 1, 0, 0.5, 0, 0,
             1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
        cv::transpose(F, F_transp);
        P = cv::Mat::eye(6, 6, CV_64FC1) * 1000;
        H = (cv::Mat_<double>(2, 6) << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0);
        cv::transpose(H, H_transp);
        R = (cv::Mat_<double>(2, 2) << 1, 0, 0, 1);

        I = cv::Mat::eye(6, 6, CV_64FC1);
        x = (cv::Mat_<double>(6, 1) << init_x, init_y, 0, 0, 0, 0);
        lastUpdate = std::chrono::steady_clock::now();
}

// TODO: add robot change position to prediction
void KalmanFilter_::prediction(const double &forVel, const double &angVel) {
        auto now = std::chrono::steady_clock::now();

        std::chrono::duration<double> diff = now - lastUpdate;
        double dt = diff.count();
        lastUpdate = now;
        cv::Mat dx = (cv::Mat_<double>(6, 1) << forVel*dt, angVel*dt, forVel, angVel, 0, 0);
        x = F * x + dx;
        P = F * P * F_transp;
}

void KalmanFilter_::update(const double &measured_x, const double &measured_y) {
        cv::Mat Y = (cv::Mat_<double>(2, 1) << measured_x, measured_y) - H * x;
        cv::Mat S = H * P * H_transp + R;

        cv::Mat K = P * H_transp * S.inv();
        x = x + K * Y;

        P = (I - K * H) * P;
}

const double &KalmanFilter_::getX() {
        return x.at<double>(0);
}
const double &KalmanFilter_::getY() {
        return x.at<double>(1);
}
