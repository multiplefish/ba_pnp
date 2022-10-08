#include "common_include.h"
#include "pixel2cam.h"

vector<double> pixel2cam(KeyPoint &kp_, cv::Mat &K_) {
    int x = kp_.x_;
    int y = kp_.y_;
    double fx = K_.at<double>(0, 0);
    double fy = K_.at<double>(1, 1);
    double cx = K_.at<double>(0, 2);
    double cy = K_.at<double>(1, 2);

    vector<double> vec_return;
    vec_return.push_back((x-cx)/fx);
    vec_return.push_back((y-cy)/fy);
    return vec_return;
}
