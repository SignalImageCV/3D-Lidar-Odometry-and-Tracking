#include "DatasetManager.hpp"

namespace Loam {

  DatasetManager::DatasetManager(string filename):m_current_index(0){
    m_source.open(filename);
  };


  PointNormalColor3fVectorCloud  DatasetManager::readMessageFromDataset(){
    BaseSensorMessagePtr msg;
    messages_registerTypes();
    Point3fVectorCloud current_point_cloud;
    if(msg=m_source.getMessage()){
      PointCloud2Message* cloud = dynamic_cast<PointCloud2Message*>(msg.get());
      if(cloud){
        cloud->getPointCloud(current_point_cloud);
      }
    }

    PointNormalColor3fVectorCloud point_cloud_with_normal;
    point_cloud_with_normal.resize( current_point_cloud.size());

    for( unsigned int i=0; i<current_point_cloud.size();++i){
      point_cloud_with_normal[i].coordinates() = current_point_cloud[i].coordinates();
      point_cloud_with_normal[i].normal() = Eigen::Vector3f::Zero();
      point_cloud_with_normal[i].color() = ColorPalette::color3fBlack();
    }

    ++m_current_index;
    return point_cloud_with_normal;
  };

}
