#include <iostream>
#include "Eigen/Core"
#include "common_include.h"
#include "KeyPoint.h"
#include "FAST_detect.h"

vector<vector<int>> GetBrief(const cv::Mat &img , VecKeyPoint &keypoints_);