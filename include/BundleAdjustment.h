#include <iostream>
#include "Match.h"


void BundleAdjustment(
  const VecVector3d &points_3d,
  const VecVector2d &points_2d,
  const cv::Mat &K,
  Sophus::SE3d &pose);