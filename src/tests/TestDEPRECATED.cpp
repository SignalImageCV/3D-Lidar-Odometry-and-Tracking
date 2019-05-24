#include "../loam/DatasetManager.hpp"

#include<iostream>

using namespace std;
using namespace srrg2_core;
using namespace Loam;

typedef struct point3d_tag{
  int index;
  cv::Point3d coords;
  point3d_tag(
    int t_index = 0,
    double t_xCoord = 0.0,
    double t_yCoord = 0.0,
    double t_zCoord = 0.0
    ):
    index( t_index),
    coords( t_xCoord, t_yCoord, t_zCoord)
  {}
}point3d;


int main( int argc, char** argv){

  messages_registerTypes();
  std::string filename = argv[1];

  MessageFileSource src;
  src.open(filename);

  int times = 0;

  BaseSensorMessagePtr msg;

  
  while( (msg=src.getMessage()) && times<1 ){
    ++times;
    PointCloud2Message* cloud = dynamic_cast<PointCloud2Message*>(msg.get());
    
    if (cloud) {
      Point3fVectorCloud current_point_cloud;
      cloud->getPointCloud(current_point_cloud);
      std::vector<point3d> points;
      points.reserve(current_point_cloud.size());
      for( int i = 0; i<current_point_cloud.size();++i){
        point3d p ( i,
            current_point_cloud[i].coordinates().x(),
            current_point_cloud[i].coordinates().y(),
            current_point_cloud[i].coordinates().z());
        points.push_back(p);
        //std::cout << "p: "<< p.index <<" x. "<<p.coords.x<<" y. "<<p.coords.y<<" z. "<<p.coords.z<<"\n";
      }
      std::cout<< "dimension of point cloud : "<< current_point_cloud.size()<<"\n";
      std::cout<< "dimension of vec of points: "<< points.size()<<"\n";
    }
  }
}




