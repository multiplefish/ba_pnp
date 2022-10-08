#include "common_include.h"
#include "KeyPoint.h"

// KeyPoint::KeyPoint(const cv::Mat &img1 , double threshold){
    

// }

Vec2 KeyPoint::getPoint(){
    Vec2 point;
    point << x_ , y_ ;
    return point;
}

