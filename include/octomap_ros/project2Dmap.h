#ifndef PROJECT2DMAP_H
#define PROJECT2DMAP_H
#include <iostream>
#include <map>
#include <cmath>
#include "myUnits.h"
#include <octomap/octomap.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "ros/ros.h"

using namespace std;

struct grid2D{
    int occupied;//1 means one node projecting into it, 2 means two nodes, ...
//    string belong;
    int a,b;
     float h;
     float g;
     float f;  //f = g+h,a*
     grid2D(){
         occupied = -1;//-1 initial /0 free /1+ occupied
     }
};

class Project2Dmap {
    double resolution;
    octomap::point3d originCoord;    

public:
    double zmin,zmax;
    Project2Dmap(const float res):resolution(res){}
    map<string,grid2D *> map_grid;
    void setOriginCoord(octomap::point3d o){
        originCoord.x()=o.x();
        originCoord.y()=o.y();
        originCoord.z()=o.z();
    }
    void setZ(double min,double max){
        zmin = min;
        zmax = max;
    }

    void XY2ab(octomap::point3d position, int & nx ,int & ny){
         nx = (int)ceil(float(abs(position.x()-originCoord.x())/resolution));
         ny = (int)ceil(float(abs(position.y()-originCoord.y())/resolution));
        nx == 0? nx =1:nx=nx;
        ny == 0? ny =1:ny=ny;
        if(position.x() < originCoord.x()){
            nx *= -1;
        }
        if(position.y() < originCoord.y()){
            ny *= -1;
        }
        if(nx>100000)
            cout<<"xy2ab error\n";
//        cout<<"nx,ny "<<nx<<","<<ny<<endl;
    }

    void ab2Morton(int a, int b, string & morton){
        string xy_belong;
        if(a>0){
            if(b>0)
                xy_belong = "A";
            else
                xy_belong = "B";
        }else{
            if(b>0)
                xy_belong = "C";
            else
                xy_belong = "D";
        }
        string m = countMorton(abs(a),abs(b));
        morton = xy_belong+ m;
    }

    void XY2Morton(octomap::point3d position, string & morton){
        string xy_belong;
        if(position.x() > originCoord.x()){
            if(position.y() >originCoord.y())
                xy_belong = "A";
            else
                xy_belong = "B";
        }else{
            if(position.y()>originCoord.y())
                xy_belong = "C";
            else
                xy_belong = "D";
        }
        int nx = (int)ceil(float(abs(position.x()-originCoord.x())/resolution));
        int ny = (int)ceil(float(abs(position.y()-originCoord.y())/resolution));
        nx == 0? nx =1:nx=nx;
        ny == 0? ny =1:ny=ny;
        string m = countMorton(nx,ny);
        morton = xy_belong+ m;
    }

    void show2Dmap(ros::Publisher marker_pub){
        int i=0;
        ros::Rate r(100);
        visualization_msgs::MarkerArray mArray;
        map<string,grid2D *>::iterator it;
        for(it = map_grid.begin();it!=map_grid.end();it++,i++){
            if((it->second)->occupied >=0){
//                double x = (it->second)->a*resolution + originCoord.x();
//                double y = (it->second)->b*resolution + originCoord.y();
                visualization_msgs::Marker m_s;
                m_s.ns  = "project2Dmap";
                m_s.header.frame_id = "/my_frame";
                m_s.header.stamp = ros::Time::now();
                m_s.id = i;
                m_s.type = visualization_msgs::Marker::CUBE;
                m_s.action = visualization_msgs::Marker::ADD;
                m_s.pose.position.x = (it->second)->a;
                m_s.pose.position.y = (it->second)->b;
//                cout<<"show-a,b"<<(it->second)->a<<","<<(it->second)->b<<endl;
                m_s.pose.position.z = 0;
                m_s.pose.orientation.x = 0;
                m_s.pose.orientation.y = 0;
                m_s.pose.orientation.z = 1;
                m_s.pose.orientation.w = 1;
                m_s.scale.x = 1;
                m_s.scale.y = 1;
                m_s.scale.z = 0.2;
                m_s.color.a = 1.0;
                if((it->second)->occupied >0){
                    m_s.color.b = 1;
                    m_s.color.r = 1;
                    m_s.color.g = 0.5;
                }else{
                    m_s.color.b = 1;
                    m_s.color.r = 0;
                    m_s.color.g = 0.5;
                }
                m_s.lifetime = ros::Duration();
                mArray.markers.push_back(m_s);
            }
        }
        if(ros::ok()){
            if (marker_pub.getNumSubscribers() == 1){
//                cout<<"show\n";
                marker_pub.publish(mArray);
            }
        }
    }
};

#endif // PROJECT2DMAP_H


















