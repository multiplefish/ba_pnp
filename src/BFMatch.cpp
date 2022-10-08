#include "common_include.h"
#include "BFMatch.h"
#include "Match.h"

vector<Match> BFMatch(vector<vector<int>> Vecvec1 , vector<vector<int>> Vecvec2){
    vector<Match> matches_;
    const int d_max = 25;//25 is ok
    const int d_min = 15;
    for (int i1 = 0; i1<Vecvec1.size(); i1++){
        if (Vecvec1[i1].empty()) continue;
        // matches[i1] = Match(0,0,0);
        Match m{i1,-1,-1};
        int dis = 256;
        for (unsigned int i2 = 0; i2 < Vecvec2.size(); i2++){

            int distance = 0;
            // int dis = 256;
            for (unsigned int j = 0; j < Vecvec1[1].size(); j++){//计算汉明距离
                int k = 0 ;
                k = Vecvec1[i1][j] ^ Vecvec2[i2][j];
                distance += k;
            }
            if (distance < d_max && distance < dis && distance >= d_min) {//目前，每一次distance小于了之前的distance都输出了。应该是要对应点的最小值的点输出才行
                dis = distance;
                m.id1_ = i1;
                m.id2_ = i2;
                m.distance_ = distance;
            }
            
        }
        if (m.id2_ == -1) {
            continue;
        }
        matches_.push_back(m);
        // cout << "matches id1,id2,distance is: " << m.id1_<<" " << m.id2_ << " " << m.distance_ << endl;
        
    }
    cout << "BFMatch is over" << endl;
    // cout << matches_.size() << endl;
    return matches_;
}