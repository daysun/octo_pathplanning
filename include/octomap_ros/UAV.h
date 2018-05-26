#ifndef UAV_H
#define UAV_H
#include "myUnits.h"
#include <vector>
#include <string>
#include <octomap/octomap.h>
#include <iostream>
using namespace std;

namespace daysun{
class UAV{
    float radius; //
    octomap::point3d position; //start
    octomap::point3d goal; //end
public:

    UAV(const float rr):radius(rr){}

    float getRadius() {return radius;}
    octomap::point3d getPosition(){return position;}
    octomap::point3d getGoal(){return goal;}

    void setPos(string s){
        vector<string> v;
        SplitString(s, v,",");
        position.x() = strToFloat(v[0]);
        position.y() = strToFloat(v[1]);
        position.z() = strToFloat(v[2]);
    }

    void setGoal(string s){
        vector<string> v;
        SplitString(s, v,",");
        goal.x() = strToFloat(v[0]);
        goal.y() = strToFloat(v[1]);
        goal.z() = strToFloat(v[2]);
    }

};
}

#endif // UAV_H
