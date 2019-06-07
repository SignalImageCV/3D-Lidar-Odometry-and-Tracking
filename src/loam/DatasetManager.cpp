#include "DatasetManager.hpp"

namespace Loam {
  DatasetManager::DatasetManager(string filename):m_current_index(0){
    m_source.open(filename);
  };


  Point3fVectorCloud  DatasetManager::readMessageFromDataset(){
    BaseSensorMessagePtr msg;
    messages_registerTypes();
    Point3fVectorCloud current_point_cloud;
    if(msg=m_source.getMessage()){
      PointCloud2Message* cloud = dynamic_cast<PointCloud2Message*>(msg.get());
      if(cloud){
        cloud->getPointCloud(current_point_cloud);
      }
    }
    ++m_current_index;
    return current_point_cloud;
  };

}
