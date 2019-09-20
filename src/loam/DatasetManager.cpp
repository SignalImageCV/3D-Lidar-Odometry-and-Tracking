#include "DatasetManager.hpp"

namespace Loam {

  DatasetManager::DatasetManager(string filename):m_current_index(0){
    messages_registerTypes();
    m_source.param_topics.value().push_back("/kitti/velo/pointcloud");
    m_source.open(filename);
  };


  PointNormalColor3fVectorCloud  DatasetManager::readMessageFromDataset(){

    BaseSensorMessagePtr msg;
    Point3fVectorCloud current_point_cloud;
    if(msg=m_source.getMessage()){
      PointCloud2MessagePtr cloud = std::dynamic_pointer_cast<PointCloud2Message>(msg);
      if(cloud){
        cloud->getPointCloud(current_point_cloud);
      }
    }



    PointNormalColor3fVectorCloud point_cloud_with_normal;
    point_cloud_with_normal.resize( current_point_cloud.size());
    current_point_cloud.copyFieldTo<0,0,PointNormalColor3fVectorCloud>(point_cloud_with_normal);

    for ( auto & p : point_cloud_with_normal){
      p.color() = ColorPalette::color3fOrange();
    }
      ++m_current_index;
    return point_cloud_with_normal;
  };


  BaseSensorMessagePtr DatasetManager::readPointerToMessageFromDataset(){
    BaseSensorMessagePtr msg;
    Point3fVectorCloud current_point_cloud;
    msg=m_source.getMessage();
    ++m_current_index;
    return msg;
  }

}
