#include <iostream>
#include "Eigen/Core"
#include "common_include.h"
#include "KeyPoint.h"
#include "FAST_detect.h"
#include "GetBrief.h"
#include "BFMatch.h"
#include "BundleAdjustment.h"
// #include "pixel2cam.h"
vector<double> pixel2cam(KeyPoint &kp_, cv::Mat &K_);

string first_file = "../data/1.png";
string second_file = "../data/2.png";
string depth_file = "../data/1_depth.png";

int main(int argc , char **argv){
    cv::Mat img1_rgb = cv::imread (first_file,1);
    cv::Mat img2_rgb = cv::imread (second_file,1);
    cv::Mat img1 = cv::imread (first_file , CV_8UC1);
    cv::Mat img2 = cv::imread (second_file ,0);
    assert(img1.data != nullptr && img2.data != nullptr); //读图
    cv::Mat d1 = cv::imread(depth_file , CV_LOAD_IMAGE_UNCHANGED);// 深度图为16位无符号数，单通道图像;此段读取深度图，并把深度储存在dd_vec中

    // cout << img1.type() << endl;
    // cout << img1.size() << endl;
    // cout << img1.rows << endl;//rows为行，cols为列
    // cout << img1.ptr<uchar>(5)[6]  << endl;

    VecKeyPoint VecKeyPoints1 = FAST_detect(img1,0.5);
    VecKeyPoint VecKeyPoints2 = FAST_detect(img2,0.5); //检测ＦＡＳＴ角点

    for(unsigned int l = 0; l < VecKeyPoints1.size(); l++){//画出特征点
        cv::Point p;
        p.x = VecKeyPoints1[l].x_;
        p.y = VecKeyPoints1[l].y_;
        cv::circle(img1_rgb, p, 3, cv::Scalar(255, 255, 0), -1);
    }
    for(unsigned int l = 0; l < VecKeyPoints2.size(); l++){
        cv::Point p;
        p.x = VecKeyPoints2[l].x_;
        p.y = VecKeyPoints2[l].y_;
        cv::circle(img2_rgb, p, 3, cv::Scalar(255, 255, 0), -1);
    }

    vector<vector<int> > Vecvec1,Vecvec2;//求出特征描述
    Vecvec1 = GetBrief (img1,VecKeyPoints1);
    Vecvec2 = GetBrief (img2,VecKeyPoints2);
    
    vector<Match> matches,matches_2;//进行BfMatch,matches是所有的匹配，包括未匹配成功的。matches2是成功匹配的。本段代码同时提取了matches_2和dd_vec
    matches = BFMatch(Vecvec1,Vecvec2);
    // cout << "size is : " << matches.size() <<  endl;
    int k = 0;
    vector<double> dd_vec;
    for (unsigned int i=0; i<matches.size(); i++){
        // int id1 = matches[i].id1_;
        int d_row = VecKeyPoints1[matches[i].id1_].y_;
        int d_col = VecKeyPoints1[matches[i].id1_].x_;

        int d2_row = VecKeyPoints2[matches[i].id2_].y_;
        int d2_col = VecKeyPoints2[matches[i].id2_].x_;

        unsigned short d = d1.ptr<unsigned short> (d_row)[d_col];
        if(d == 0 ){ //去除图像中深度为0的点
            continue;
        }
        double dd = d / 5000.0;
        // cout << "图１的匹配点坐标，及深度 "<< d_col << "," << d_row << ":" << dd << endl;
        // cout << "图2的匹配点坐标，及深度  "<< d2_col << "," << d2_row << endl;

        dd_vec.push_back(dd);
        matches_2.push_back(matches[i]);
        k += 1;
    }
    cout << "一共找到了" << matches_2.size() << "组匹配点" << endl;



    cv::Mat combine;//绘制combine匹配成功图像
    cv::hconcat(img1_rgb,img2_rgb,combine); //图像合并函数
    for (unsigned int j=0; j<matches_2.size(); j++){//绘制匹配点对之间的连线
        cv::Point p1 = cv::Point(VecKeyPoints1[matches_2[j].id1_].x_ , VecKeyPoints1[matches_2[j].id1_].y_ );
        cv::Point p2 = cv::Point(VecKeyPoints2[matches_2[j].id2_].x_ + 640 , VecKeyPoints2[matches_2[j].id2_].y_);
        // cout << j <<"hh:" << VecKeyPoints1[matches_2[j].id1_].x_ << " " << VecKeyPoints1[matches_2[j].id1_].y_  << endl;
        cv::line(combine, p1, p2, cv::Scalar(200, 0, 0), 1); //BGR成像
        
    }

    VecVector3d pts3d; //进行BA
    VecVector2d pts2d;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    Sophus::SE3d pose_gn;
    for (auto i=0; i<dd_vec.size(); i++){
        double dd = dd_vec[i];
        auto id1 = matches_2[i].id1_;
        auto id2 = matches_2[i].id2_;
        KeyPoint pixel = VecKeyPoints1[id1];
        vector<double> cam = pixel2cam(pixel,K);

        pts3d.push_back(Eigen::Vector3d(cam[0]*dd , cam[1]*dd , dd));
        pts2d.push_back(Eigen::Vector2d( VecKeyPoints2[id2].x_ , VecKeyPoints2[id2].y_ ));// 没毛病
        // cout << i << ":" << pts2d[i].transpose() << endl;
    }
    BundleAdjustment(pts3d,pts2d,K,pose_gn);


    cv::imshow("img2_rgb",img2_rgb);
    cv::imshow("img1_rgb",img1_rgb);
    cv::imshow("combine",combine);
    // cv::imwrite("../data/combine_gray.png",combine);
    cv::waitKey(0);

    return 0;
}

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


