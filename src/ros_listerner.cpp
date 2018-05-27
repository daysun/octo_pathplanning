#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <assert.h>
#include <cstddef>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "octomap_ros/Id_PointCloud2.h"
#include "octomap_ros/loopId_PointCloud2.h"
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
//octomap
#include "octomap_ros/project2Dmap.h"
#include "octomap_ros/ColorOcTree.h"
#include "octomap_ros/UAV.h"
//#include <thread>
#include "octomap_ros/GlobalPlan.h"
using namespace std;

///receive data(orb-slam published it) from ros
/// saved it as sample.ot
/// project the map into 2D layer--fixed height
/// path planning
octomap::ColorOcTree tree( 0.05 );
int loopNum = -1;
bool isFirstSet = false;
Project2Dmap * pMap= new Project2Dmap(tree.getResolution());
double zmin = 0.1,zmax = 2.7;//get it from orb-slam
ros::Publisher marker_pub,route_pub;
daysun::UAV * robot =  new daysun::UAV(0.15); //r=0.15
daysun::AstarPlanar * globalPlanr = new daysun::AstarPlanar();

//for costmap and pathplanning
//daysun::AstarPlanar * astar;
//thread* pathPlan;

//ofstream outfile("/home/daysun/GlobalTime.txt", ofstream::app);
//ros::Duration bTcreate;

int countKF = 2;
///initial insert
void chatterCallback(const octomap_ros::Id_PointCloud2::ConstPtr & my_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(my_msg->msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);   
    if(!isFirstSet){
        pMap->setOriginCoord(octomap::point3d(temp_cloud->points[0].x,temp_cloud->points[0].y,temp_cloud->points[0].z));
        pMap->setZ(zmin,zmax);
        isFirstSet = true;
    }
    for (int i=0;i<temp_cloud->points.size();i++)
    {
        tree.updateNode(octomap::point3d(temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z),true);        
     }

    for  (int i=0;i<temp_cloud->points.size();i++)
    {
        tree.IntegrateNodeColor( temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z,
                                 temp_cloud->points[i].r,temp_cloud->points[i].g,temp_cloud->points[i].b);
        tree.integrateNodeId(temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z,
                             my_msg->kf_id);
    }

    countKF +=1;
    if(countKF == 20){
        cout<<"updateInnerOccupancy + pruneTree every 20KF\n";
    tree.updateInnerOccupancy();
    tree.pruneTree(tree.getRoot(), 0);
    countKF = 2;
    }

    ///project into 2D
    tree.projector2D(pMap);
    pMap->show2Dmap(marker_pub,robot);

    ///2D map path planning
    if(pMap->pp == false){
        if(pMap->checkGoal(robot) && pMap->checkPos(robot) ){
            pMap->pp = true;//find road-pp-true/not pp-false
//            cout<<"compose a new map-once\n";
//            Project2Dmap * tempMap = new Project2Dmap(*pMap);
//            cout<<"copy over\n";
            if(globalPlanr->findRoute(pMap,robot)){
                pMap->pp = true;
                cout<<"find road\n";
                if(route_pub.getNumSubscribers()){
                    globalPlanr->showRoute(pMap,route_pub,robot->getRadius());
                }
            }else{
                cout<<"not find road\n";
                pMap->pp = false;
            }
        }
    }
}

int num=0;
///after ORB-SLAM local update
void chatterCallback_local(const octomap_ros::Id_PointCloud2::ConstPtr & my_msg)
{
    //delete accord to id
//    cout<<"change:"<<my_msg->kf_id<<endl;
    tree.deleteById(my_msg->kf_id); //0.02
    num++;
    if(num == 20){
        tree.updateProjectorMap(pMap);
        num=0;
    }
//    tree.updateInnerOccupancy();
//    tree.pruneTree(tree.getRoot(), 0);

    //add new pointCloud-0.03
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(my_msg->msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    for (int i=0;i<temp_cloud->points.size();i++)
    {
        tree.updateNode(octomap::point3d(temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z),true);
    }

    for  (int i=0;i<temp_cloud->points.size();i++)
    {
        tree.IntegrateNodeColor( temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z,
                                 temp_cloud->points[i].r,temp_cloud->points[i].g,temp_cloud->points[i].b);
        tree.integrateNodeId(temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z,
                             my_msg->kf_id);
    }
//    tree.projector2D(pMap);
//    pMap->show2Dmap(marker_pub,robot);

    ///after local update
    /// check if road was affected by this update
    if(pMap->pp == true)
        if(globalPlanr->getRoadSize() >0){
            if(!globalPlanr->checkRoadFeasibility(pMap,robot->getRadius())){ //true-ok/false-replan
                cout<<"updating affects road,replaning...\n";
                pMap->pp = false;
            }
        }
}

///after ORB-SLAM global update
void chatterCallback_global(const octomap_ros::loopId_PointCloud2::ConstPtr & my_msg){
    ros::Time tGlobal1 = ros::Time::now();
    //delete the old tree
    if(loopNum == -1){
        //the first time
        tree.deleteTree();
        loopNum = my_msg->loop_id;
        cout<<"global-delete tree,new one\n";
    }else{
        //not the first time
        if(loopNum != my_msg->loop_id){
            tree.deleteTree();
            loopNum = my_msg->loop_id;
        }
    }

//    cout<<"global:"<<my_msg->kf_id<<",loopNum:"<<loopNum<<endl;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(my_msg->msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    for (int i=0;i<temp_cloud->points.size();i++)
    {
        tree.updateNode(octomap::point3d(temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z),true);
     }

    for  (int i=0;i<temp_cloud->points.size();i++)
    {
        tree.IntegrateNodeColor( temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z,
                                 temp_cloud->points[i].r,temp_cloud->points[i].g,temp_cloud->points[i].b);
        tree.integrateNodeId(temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z,
                             my_msg->kf_id);
    }
//            ros::Time tGlobal2 = ros::Time::now();
//         bTcreate = bTcreate+(tGlobal2-tGlobal1);
}

void coutInnerOccupancy(octomap::ColorOcTreeNode* node, unsigned int depth,int tree_depth) {
  if (node->hasChildren()){
    if (depth < tree_depth){
      for (unsigned int i=0; i<8; i++) {
        if (node->childExists(i)) {
          coutInnerOccupancy(node->getChild(i), depth+1,tree_depth);
        }
      }
    }
    cout<<node->getLogOdds()<<"\n";
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ORB_SLAM2_listener");
  ros::start();

  ros::NodeHandle n;
//  bTcreate=  ros::Time::now()-  ros::Time::now();
  string pos;
  string goal;
  ros::param::get("~pos",pos);
  ros::param::get("~goal",goal);
  robot->setPos(pos);
  robot->setGoal(goal);//new a robot


//  astar = new daysun::AstarPlanar();
//  pathPlan = new thread(&daysun::AstarPlanar::Run,astar);

  marker_pub = n.advertise<visualization_msgs::MarkerArray>("twoDMap_marker_array", 1000);
  route_pub= n.advertise<visualization_msgs::MarkerArray>("route_marker_array", 1000);

  ros::Subscriber sub = n.subscribe("/ORB_SLAM/pointcloud2", 1000, chatterCallback);
  ros::Subscriber sub_change = n.subscribe("ORB_SLAM/pointcloudlocalup2", 1000, chatterCallback_local);
  ros::Subscriber sub_global = n.subscribe("ORB_SLAM/pointcloudup2", 1000, chatterCallback_global);  

  ros::spin();
  ros::shutdown();
//  std::cout << "time cost all: " << allTime.toSec() << std::endl; //hpcl_office_asuse.bag-34.388

  //test updateOccupancyChildren--test the children's log-odds
  //int tree_depth = tree.getTreeDepth();
  //coutInnerOccupancy(tree.getRoot(),0,tree_depth);

  //count global time
//  outfile<<bTcreate.toSec()*1000<<"\t";  //micro sec
//  cout<<"global time:"<<bTcreate.toSec()*1000<<endl;

  tree.updateInnerOccupancy();
  tree.pruneTree(tree.getRoot(), 0);
  tree.write( "/home/daysun/rros/tesst.ot" );
  cout<<"write done."<<endl;

  return 0;
}
