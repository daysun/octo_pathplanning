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
#include "UAV.h"
#include "float.h"
#include "math.h"

using namespace std;

struct grid2D{
    int occupied;//1 means one node projecting into it, 2 means two nodes, ...
//    string belong;
    int a,b;
     float h;
     float g; //real cost
     float f;  //f = g+h,a*
     grid2D * father;
     grid2D(){
         occupied = -1;//-1 initial /0 free /1+ occupied
         father = NULL;
         a=0;
         b=0;
     }
};

class Project2Dmap {
    double resolution;
    octomap::point3d originCoord;    
    float TravelCost(octomap::point3d cur,octomap::point3d des){
        return sqrt(pow(cur.x()-des.x(),2) + pow(cur.y()-des.y(),2));
    }

    bool isContainedQ(grid2D * s,  list<grid2D *> & Q){
        list<grid2D *>::iterator it = Q.begin();
        while(it != Q.end()){
            if((s->a == (*it)->a) && (s->b == (*it)->b))
                return true;
            it++;
        }
        return false;
    }    

    bool checkNeighbor(int a,int b,int r){
        for(int i=a-r;i<=a+r;i++){
            for(int j=b-r;j<=b+r;j++){
                string mor;
                ab2Morton(i,j,mor);
                int count = map_grid.count(mor);
                if(count!=0){//find
                    if( ((map_grid.find(mor))->second)->occupied != 0 ){
                        return false;
                    }
                }else{
                    return false;
                }//unknown-cant go there
            }
        }
        return true;
    }

public:
    double zmin,zmax;
    Project2Dmap(const float res):resolution(res){
        find_goal = false;
        find_pos = false;
        pp = false;
    }
    Project2Dmap(const Project2Dmap& m){
        resolution = m.resolution;
        originCoord.x() = m.originCoord.x();
        originCoord.y() = m.originCoord.y();
        originCoord.z() = m.originCoord.z();
        zmin = m.zmin;
        zmax = m.zmax;
        find_goal = m.find_goal;
        find_pos = m.find_pos;
        pp = m.pp;
        map<string,grid2D *>::const_iterator it;
        for(it=m.map_grid.begin();it!=m.map_grid.end();it++){
            map_grid.insert(make_pair(it->first,it->second));
        }
    }

    map<string,grid2D *> map_grid;
    bool find_goal;
    bool find_pos;
    bool pp;
    void setOriginCoord(octomap::point3d o){
        originCoord.x()=o.x();
        originCoord.y()=o.y();
        originCoord.z()=o.z();
    }
    void setZ(double min,double max){
        zmin = min;
        zmax = max;
    }

    void XY2ab(octomap::point3d & position, int & nx ,int & ny){
         nx = (int)ceil(float(abs(position.x()-originCoord.x())/resolution));
         ny = (int)ceil(float(abs(position.y()-originCoord.y())/resolution));
//        nx == 0? nx =1:nx=nx;
//        ny == 0? ny =1:ny=ny;
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

    void XY2Morton(double x,double y, string & morton){
        string xy_belong;
        if(x > originCoord.x()){
            if(y >originCoord.y())
                xy_belong = "A";
            else
                xy_belong = "B";
        }else{
            if(y>originCoord.y())
                xy_belong = "C";
            else
                xy_belong = "D";
        }
        int nx = (int)ceil(float(abs(x-originCoord.x())/resolution));
        int ny = (int)ceil(float(abs(y-originCoord.y())/resolution));
//        nx == 0? nx =1:nx=nx;
//        ny == 0? ny =1:ny=ny;
        string m = countMorton(nx,ny);
        morton = xy_belong+ m;
    }

    bool checkGoal(daysun::UAV * robot){
        if(robot->getGoal().z()<=zmax && robot->getGoal().z()>=zmin){
            string morton;
            XY2Morton(robot->getGoal().x(),robot->getGoal().y(),morton);
            //find the grid-initial
            if(map_grid.count(morton)==0){
                return false;
            }else{
                cout<<"find goal\n";
                map<string,grid2D *>::iterator it = map_grid.find(morton);
                if(AccessibleNeighbors(it->second,robot->getRadius()).size() >0)
                    return true;
                else
                    return false;
            }
        }else{
            cout<<"test-goal error, out of range\n";
            cout<<"z,zmax,zmin "<<robot->getGoal().z()<<","<<zmax<<","<<zmin<<endl;
            return false;
        }
    }

    bool checkPos(daysun::UAV * robot){
        if(robot->getPosition().z()<=zmax && robot->getPosition().z()>=zmin){
            string morton;
            XY2Morton(robot->getPosition().x(),robot->getPosition().y(),morton);
            //find the grid-initial
            if(map_grid.count(morton)==0){
                return false;
            }else{
//                cout<<"find pos\n";
                map<string,grid2D *>::iterator it = map_grid.find(morton);
                if(AccessibleNeighbors(it->second,robot->getRadius()).size() >0)
                    return true;
                else
                    return false;
            }
        }else{
            cout<<"test-pos error, out of range\n";
            cout<<"z,zmax,zmin "<<robot->getPosition().z()<<","<<zmax<<","<<zmin<<endl;
            return false;
        }
    }

    void show2Dmap(ros::Publisher marker_pub,daysun::UAV * robot){
        int i=2;
        ros::Rate r(50);
        //for goal
        visualization_msgs::MarkerArray mArray;
        {
            visualization_msgs::Marker m;
            m.ns  = "project2Dmap";
            m.header.frame_id = "/my_frame";
            m.header.stamp = ros::Time::now();
            m.id = 0;
            m.type = visualization_msgs::Marker::SPHERE;
            m.action = visualization_msgs::Marker::ADD;
            int a,b;
            octomap::point3d temp(robot->getGoal().x(),robot->getGoal().y(),robot->getGoal().z());
            XY2ab(temp,a,b);
            m.pose.position.x = a;
            m.pose.position.y = b;
            m.pose.position.z = 0;
            m.pose.orientation.x = 0;
            m.pose.orientation.y = 0;
            m.pose.orientation.z = 1;
            m.pose.orientation.w = 1;
            m.scale.x = (int)ceil(robot->getRadius()*2/resolution);
            m.scale.y = (int)ceil(robot->getRadius()*2/resolution);
            m.scale.z = 0.8;
            m.color.a = 1.0;
            m.color.b = 0;
            m.color.r = 1;
            m.color.g = 1;
            m.lifetime = ros::Duration();
            mArray.markers.push_back(m);

        }
        //for pos
        {
            visualization_msgs::Marker m;
            m.ns  = "project2Dmap";
            m.header.frame_id = "/my_frame";
            m.header.stamp = ros::Time::now();
            m.id = 1;
            m.type = visualization_msgs::Marker::SPHERE;
            m.action = visualization_msgs::Marker::ADD;
            int a,b;
            octomap::point3d temp(robot->getPosition().x(),robot->getPosition().y(),robot->getPosition().z());
            XY2ab(temp,a,b);
            m.pose.position.x = a;
            m.pose.position.y = b;
            m.pose.position.z = 0;
            m.pose.orientation.x = 0;
            m.pose.orientation.y = 0;
            m.pose.orientation.z = 1;
            m.pose.orientation.w = 1;
            m.scale.x = (int)ceil(robot->getRadius()*2/resolution);
            m.scale.y = (int)ceil(robot->getRadius()*2/resolution);
            m.scale.z = 0.8;
            m.color.a = 1.0;
            m.color.b = 1;
            m.color.r = 1;
            m.color.g = 1;
            m.lifetime = ros::Duration();
            mArray.markers.push_back(m);

        }
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

    void showGridList(ros::Publisher marker_pub,list<grid2D *> & path,double rr){
        ros::Rate r(50);
        int i =0;
        visualization_msgs::MarkerArray mArray;
        list<grid2D *>::iterator it;
        for(it = path.begin();it!= path.end();it++,i++){
            visualization_msgs::Marker m_s;
            m_s.ns  = "project2Dmap";
            m_s.header.frame_id = "/my_frame";
            m_s.header.stamp = ros::Time::now();
            m_s.id = i;
            m_s.type = visualization_msgs::Marker::SPHERE;
            m_s.action = visualization_msgs::Marker::ADD;
            m_s.pose.position.x = (*it)->a;
            m_s.pose.position.y = (*it)->b;
            m_s.pose.position.z = 0;
            m_s.scale.x = (int)ceil(rr*2/resolution);
            m_s.scale.y = (int)ceil(rr*2/resolution);
            m_s.scale.z = 2;
            m_s.pose.orientation.x = 0;
            m_s.pose.orientation.y = 0;
            m_s.pose.orientation.z = 1;
            m_s.pose.orientation.w = 1;
            m_s.color.a = 1.0;
            m_s.color.b = 0;
            m_s.color.r = 1;
            m_s.color.g = 1;
            m_s.lifetime = ros::Duration();
            mArray.markers.push_back(m_s);
        }

        if(ros::ok()){
            if (marker_pub.getNumSubscribers() == 1){
                marker_pub.publish(mArray);
            }
        }
    }

    //true-collide, false-no collide
    //due to the zmin-zmax contains the height of robot
    //here we can simply consider the value of occupied
    bool CollisionCheck(grid2D * slope,float rr){
        int r = ceil(rr/resolution);//radius
        int a = slope->a;
        int b = slope->b;
        for(int i=a-r;i<=a+r;i++){
            for(int j=b-r;j<=b+r;j++){
                string mor;
                ab2Morton(i,j,mor);
                int count = map_grid.count(mor);
                if(count!=0){//find
                    if( ((map_grid.find(mor))->second)->occupied >0 ){
                        //collide
                        return true;
                    }
                }else{//unknown
                }
            }
        }
        return false;
    }

    list<grid2D *>  AccessibleNeighbors(grid2D * slope,float rr){
        list<grid2D *> neighbor;
        int r = ceil(rr/resolution);
        int a0 = slope->a;
        int b0 = slope->b;
        for(int a=a0-1;a<=a0+1;a++)
            for(int b=b0-1;b<=b0+1;b++){
                if(checkNeighbor(a,b,r)){
                    string m;
                    ab2Morton(a,b,m);
                    neighbor.push_back((map_grid.find(m))->second);
                }
            }
        return neighbor;
    }

    void computeCost(daysun::UAV * robot){
        int pos_a,pos_b;
        octomap::point3d temp(robot->getPosition().x(),robot->getPosition().y(),robot->getPosition().z());
        XY2ab(temp,pos_a,pos_b);
         list<grid2D *> Q; //list of cell to be computed
         list<grid2D *> closed;//no checking again
         list<grid2D *> traversability; //can travel
         string morton;
         XY2Morton(robot->getGoal().x(),robot->getGoal().y(),morton);
         //find the grid-initial
         if(map_grid.count(morton)==0){
             cout<<"cant find goal grid\n";
             find_goal = false;
             return;
         }else{
             map<string,grid2D *>::iterator it = map_grid.find(morton);
             Q.push_back(it->second);
         }

         while(Q.size()!=0){
             if(!CollisionCheck(Q.front(),robot->getRadius())){
                 //no collision
                 list<grid2D *> neiGrid = AccessibleNeighbors(Q.front(),robot->getRadius());
                 list<grid2D *>::iterator itN;
                 for(itN = neiGrid.begin();itN != neiGrid.end();itN++){
                     octomap::point3d q,itn;
                     q.x() = Q.front()->a;
                     q.y() = Q.front()->b;
                     itn.x() = (*itN)->a;
                     itn.y() = (*itN)->b;
                     q.z()=itn.z()=0;
                     if((*itN)->h > Q.front()->h + TravelCost(q,itn)){
                         (*itN)->h = Q.front()->h + TravelCost(q,itn);
                         if(!isContainedQ(*itN,Q) && !isContainedQ(*itN,closed) && !isContainedQ(*itN,traversability)){
                             Q.push_back(*itN);
                             //check if_pos
                             if(((*itN)->a==pos_a) && ((*itN)->b == pos_b)){
                                 find_pos = true;
                                 cout<<"find pos,break\n";
                                 break;
                             }
                         }
                     }
                 }
                 cout<<"out of for\n";
                 traversability.push_back(Q.front());
             }else{
                 Q.front()->h = FLT_MAX;//collide
                 closed.push_back(Q.front());
             }
             Q.pop_front();
         }
         cout<<"out of while\n";
         cout<<"cost traversability.size()"<<endl;
    }
};

#endif // PROJECT2DMAP_H


















