#ifndef _MATCH_H_
#define _MATCH_H_
#include "common_include.h"

class Match{
public:
    int id1_;
    int id2_;
    int distance_;

    Match();
    Match(int x , int y , int distance):
    id1_(x) , id2_(y) , distance_(distance) {}

    ~Match(){}

    Vec2 getPoint(){
            Vec2 point;
        point << id1_ , id2_ ;
        return point;
    }

};

#endif