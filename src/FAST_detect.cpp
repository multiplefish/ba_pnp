#include <iostream>
#include "Eigen/Core"
#include "common_include.h"
#include "KeyPoint.h"
#include "FAST_detect.h"
VecKeyPoint FAST_detect(const cv::Mat &img , double percent){
    VecKeyPoint keypoints_;
    int keyPoint_num = 0;
    // KeyPoint keypoint_(1.2,2.0,123,5);
    int mask[16*2] = {
        -3,0,
        -3,1,
        -2,2,
        -1,3,
        0,3,
        1,3,
        2,2,
        3,1,
        3,0,
        3,-1,
        2,-2,
        1,-3,
        0,-3,
        -1,-3,
        -2,-2,
        -3,-1
    };
    vector<int> Vecfeature;
    int bigger,smaller,equal;
    
    for (int i = 20; i < img.rows-20; i++){
        for (int j = 20; j < img.cols-20; j++){ //图像　排除边缘五像素的点
            smaller = bigger = equal = 0;
            for (int k=0; k<16; k++){//周围１６个点，有15个大于或小于ｐｅｒｃｅｎｔ*像素值，则为角点
                if(img.ptr<uchar>(i)[j] - img.ptr<uchar>(i+mask[2*k])[j+mask[2*k+1]] >= percent * img.ptr<uchar>(i)[j] ){
                    bigger += 1;
                }
                if(img.ptr<uchar>(i)[j] - img.ptr<uchar>(i+mask[2*k])[j+mask[2*k+1]] <= -percent * img.ptr<uchar>(i)[j] ){
                    smaller += 1;
                } 
                else equal += 1;
            }

            if (bigger >= 14){ //邻域16个点中同时有１５个大于或小于目标点则认为是特征点（取巧的算法）//14 and 25;15 and 30
                keyPoint_num += 1;
                KeyPoint keypoint_(j,i,keyPoint_num,bigger);
                // cout << keypoint_.getPoint() << endl;
                keypoints_.push_back(keypoint_);
            }
            if (smaller >= 14){
                keyPoint_num += 1;
                KeyPoint keypoint_(j,i,keyPoint_num,smaller);
                // cout << keypoint_.getPoint() << endl;
                keypoints_.push_back(keypoint_);
            }
            
            
        }
    }
    cout << "FAST Detect is over" << endl;
    return keypoints_;


}