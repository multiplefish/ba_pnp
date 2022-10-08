#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;
using namespace Eigen;

typedef vector<uint32_t> DescType;
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

typedef Eigen::Matrix<double, 3, 4> Mat34;
typedef Eigen::Matrix<double, 2, 1> Vec2;

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;