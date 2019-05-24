#include "DatasetManager.hpp"

namespace Loam {
  DatasetManager::DatasetManager(string filename):m_current_index(0){
    m_source.open(filename);
  };


  vector<ScanPoint> DatasetManager::readMessageFromDataset(){
    BaseSensorMessagePtr msg;
    messages_registerTypes();
    vector<ScanPoint> scanPoints;
    if(msg=m_source.getMessage()){
      PointCloud2Message* cloud = dynamic_cast<PointCloud2Message*>(msg.get());
      if(cloud){
        Point3fVectorCloud current_point_cloud;
        cloud->getPointCloud(current_point_cloud);
        scanPoints.reserve( current_point_cloud.size());
        for(unsigned int i = 0; i<current_point_cloud.size();++i){
          ScanPoint p ( m_current_index, i,
            current_point_cloud[i].coordinates().x(),
            current_point_cloud[i].coordinates().y(),
            current_point_cloud[i].coordinates().z());
          scanPoints.push_back(p);
        }
      }
    }
    ++m_current_index;
    return scanPoints;
  };

}
