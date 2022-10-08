#include "common_include.h"
#include "BundleAdjustment.h"


void BundleAdjustment(
  const VecVector3d &points_3d,
  const VecVector2d &points_2d,
  const cv::Mat &K,
  Sophus::SE3d &pose)//传入P点，P点像素在I2投影，内参;初始pose,在此程序中初始pose为0;
  {
  cout << "*************start BA***************" << endl;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  const int iterations = 10;
  double cost = 0, lastCost = 0;
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  for (int iter = 0; iter < iterations; iter++) {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Vector6d b = Vector6d::Zero();

    cost = 0;
    // compute cost
    for (int i = 0; i < points_3d.size(); i++) {
      Eigen::Vector3d pc = pose * points_3d[i];//pose即待优化的运动的位姿，pc是P撇，即变换到I2相机坐标系下的空间点P.乘法进行了重载
      // cout << "pose is :"<< pose.matrix() << endl;
      
      double inv_z = 1.0 / pc[2];
      double inv_z2 = inv_z * inv_z;
      Eigen::Vector2d proj (fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);//P点做投影到I2平面，存入proj中

      Eigen::Vector2d e = points_2d[i] - proj;//作差，得到差值;是观测值-预测值！

      cost += e.squaredNorm();//误差的二范数的平方
      Eigen::Matrix<double, 2, 6> J;//2*6的雅克比矩阵，即误差相对于位姿求导，通过导数可以知道有了误差以后我们应该往哪个方向去优化
      J << -fx * inv_z,
        0,
        fx * pc[0] * inv_z2,
        fx * pc[0] * pc[1] * inv_z2,
        -fx - fx * pc[0] * pc[0] * inv_z2,
        fx * pc[1] * inv_z,
        0,
        -fy * inv_z,
        fy * pc[1] * inv_z,
        fy + fy * pc[1] * pc[1] * inv_z2,
        -fy * pc[0] * pc[1] * inv_z2,
        -fy * pc[0] * inv_z;

      H += J.transpose() * J;//高斯牛顿方法，H*dx=b
      b += -J.transpose() * e;
    }

    Vector6d dx;
    dx = H.ldlt().solve(b);//对H做LDLT分解，并求解dx
    // dx = H.qr().solve(b);//对H做QR分解，并求解 //显然不行
    // cout <<"dx:" << endl << dx <<endl;
    // cout <<"dx.norm():" << endl << dx.norm() <<endl;

    if (isnan(dx[0])) {
      cout << "result is nan!" << endl;
      break;
    }

    if (iter > 0 && cost >= lastCost) {
      // cost increase, update is not good//发散的情况
      cout << "final cost: " << cost << endl;
      break;
    }

    // update your estimation
    pose = Sophus::SE3d::exp(dx) * pose;
    lastCost = cost;
    // cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    cout.precision(10);
    cout << "iteration " << iter << " cost=" << cost << endl;

    if (dx.norm() < 1e-6) {
      // converge 当dx.norm<1e-6时停止迭代
      break;
    }
  }

  cout << "pose by g-n: \n" << pose.matrix() << endl;
}
