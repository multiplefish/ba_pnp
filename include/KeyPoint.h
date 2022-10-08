#ifndef _KeyPoint_H_
#define _KeyPoint_H_
#include "common_include.h"

class KeyPoint {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // typedef std::shared_ptr<KeyPoint> Ptr;

    int x_,y_;
    int id_;
    int num_;//周围一圈亮度差别大的点的个数，用来筛选，防止角点过于集中。
    int pyramid_ = -1;//金字塔层数

    KeyPoint();
    KeyPoint(int x , int y , int id , int num ):
    x_(x) , y_(y) , id_(id) , num_(num){}

    KeyPoint(int x , int y , int id , int num , int pyramid): //有金字塔的构造
    x_(x) , y_(y) , id_(id) , num_(num) , pyramid_(pyramid){}

    // KeyPoint(const cv::Mat &img1 , double threshold = 10.0 );

    long int getID(){
        return id_;
    }

    Vec2 getPoint();

    



};

#endif