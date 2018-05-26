#ifndef GLOBALPLAN_H
#define GLOBALPLAN_H
#include<queue>
#include <octomap/octomap.h>
#include "project2Dmap.h"
#include "omp.h"
using namespace std;

namespace daysun{
struct MyCompare {
  bool operator()( float k1,  float k2) {
      return  k1<k2;
  }
};

class AstarPlanar{
    multimap<float,grid2D *,MyCompare> open_queue;
    list<grid2D *> closed_list;
    list<grid2D *> global_path;
//    octomap::point3d start,goal;
    bool isContainedClosed(grid2D * s,list<grid2D *> & closed_list){
        list<grid2D *>::iterator it = closed_list.begin();
        while(it != closed_list.end()){
            if(((*it)->a == s->a) && ((*it)->b == s->b) )
                return true;
            it++;
        }
        return false;
    }

    bool isContaninedOpen(grid2D * s,multimap<float,grid2D *,MyCompare>  & open_queue,
                          multimap<float,grid2D *,MyCompare>::iterator & iTemp){
        multimap<float,grid2D *,MyCompare>::iterator it = open_queue.begin();
        while(it != open_queue.end()){
//            if((it->first) != s->f)
//                break;
            if(((it->second)->a == s->a) && ((it->second)->b == s->b)){
                iTemp = it;
                return true;
            }
            it++;
        }
        return false;
    }

    double countH(grid2D * s,int ag,int bg){
//        return sqrt(pow(s->a-ag,2) + pow(s->b-bg,2));
        return abs(s->a-ag)+abs(s->b-bg);
    }

    double TravelCost(grid2D * s1,grid2D * s2){
        return sqrt(pow(s1->a-s2->a,2) + pow(s1->b-s2->b,2));
    }

public:
    bool findRoute(Project2Dmap * map2D,daysun::UAV * robot){
//        double time_start3 = stopwatch();
        //find where the start is
        int ag,bg;
        string mt_pos;
        octomap::point3d goal_tmp(robot->getGoal().x(),robot->getGoal().y(),0);
        bool route  = false;
        map2D->XY2ab(goal_tmp,ag,bg);
        map2D->XY2Morton(robot->getPosition().x(),robot->getPosition().y(),mt_pos);
        //initial
        map<string,grid2D *>::iterator it = map2D->map_grid.find(mt_pos);
        if(it != map2D->map_grid.end())
            if(!map2D->CollisionCheck(it->second,robot->getRadius())){
                it->second->g = 0;//start actual
                it->second->h = countH(it->second,ag,bg);
                it->second->f = it->second->g + it->second->h;
                open_queue.insert(make_pair(it->second->f,it->second));

                while(open_queue.size() != 0){
//                    cout<<"open size "<<open_queue.size()<<endl;
                    multimap<float,grid2D *,MyCompare>::iterator it_Open = open_queue.begin();
                    grid2D * temp = it_Open->second; //min f
                    //if find the goal
                    if((temp->a == ag) && (temp->b == bg)){
                        route = true;
                        global_path.push_front(temp);
                        cout<<"found the route to goal\n";
                        break;
                    }                    
                    //find neighbors
                    list<grid2D *> neighborGrids = map2D->AccessibleNeighbors(temp,robot->getRadius());
                    list<grid2D *>::iterator itS;
//                    #pragma omp parallel for
                    for(itS = neighborGrids.begin();itS!= neighborGrids.end();itS++){
                        if(!map2D->CollisionCheck((*itS),robot->getRadius())){
                            multimap<float,grid2D *,MyCompare>::iterator  itTemp;
                            //for each traversible neighbors
                            //check if find goal
                            if(((*itS)->a == ag) && ((*itS)->b == bg) ){
                                (*itS)->f = temp->f+0.01; //for test
                                (*itS)->father = temp;
                                open_queue.insert(make_pair( (*itS)->f ,*itS));
                                break;
                            }                            
                            //f=g+h
                            if(isContainedClosed(*itS,closed_list)){
                                //do nothing
                            }else if (isContaninedOpen(*itS,open_queue,itTemp)){
                                if((*itS)->g > temp->g + TravelCost(*itS,temp)){
                                    //remove and insert
                                    //update g and f, and father node
                                    (*itS)->g = temp->g + TravelCost(*itS,temp);
                                    (*itS)->h = countH(*itS,ag,bg);
                                    (*itS)->f = (*itS)->g + (*itS)->h;
                                    (*itS)->father = temp;
                                    open_queue.erase(itTemp);
                                    open_queue.insert(make_pair( (*itS)->f ,*itS));
                                }
                            }else{
                                //update g and f, and father node
                                //insert into OPEN
                                 (*itS)->g = temp->g + TravelCost(*itS,temp);
                                 (*itS)->h = countH(*itS,ag,bg);
                                 (*itS)->f = (*itS)->g + (*itS)->h;
                                 (*itS)->father = temp;
                                 open_queue.insert(make_pair( (*itS)->f ,*itS));
                            }
                        }
                    }//for end
                    closed_list.push_back(temp);
                    open_queue.erase(it_Open);
                }//end while
            }

        if(route){
            grid2D * i = global_path.front();
            while(i->father != NULL){
                global_path.push_front(i->father);
                i = global_path.front();
            }
//            double time_end3 = stopwatch();
//            cout<<"Global planr done. A*: "<<(time_end3-time_start3)<<" s\n";
            cout<<"path length "<<global_path.size()<<endl;
            return true;
        }else{
            cout<<"not find the road\n";
            return false;
        }
    }

    void showRoute(Project2Dmap * map2D,ros::Publisher marker_pub,float rr){
        map2D->showGridList(marker_pub,global_path,rr);
        cout<<"route show done\n";
    }

};
}

#endif // GLOBALPLAN_H
