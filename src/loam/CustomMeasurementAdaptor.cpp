#include "CustomMeasurementAdaptor.hpp"

using namespace std;
using namespace srrg2_core;

namespace Loam{

  void CustomMeasurementAdaptor::compute() {


    const sphericalImage_params params(
      param_num_vertical_rings.value(),
      param_num_points_ring.value(),
      param_epsilon_times.value(),
      param_epsilon_radius.value(),
      param_depth_differential_threshold.value(),
      param_min_neighboors_for_normal.value(), 
      param_epsilon_c.value(),
      param_epsilon_d.value(),
      param_epsilon_n.value(),
      param_epsilon_l.value(),
      param_epsilon_dl.value(),
      param_epsilon_p.value(),
      param_epsilon_dp.value()
    );

    Point3fVectorCloud current_point_cloud;
    PointCloud2MessagePtr cloud = std::dynamic_pointer_cast<PointCloud2Message>(_measurement);
    if(cloud){
      cloud->getPointCloud(current_point_cloud);
    }
    PointNormalColor3fVectorCloud point_cloud_with_normal;
    point_cloud_with_normal.resize( current_point_cloud.size());
    current_point_cloud.copyFieldTo<0,0,PointNormalColor3fVectorCloud>(point_cloud_with_normal);


    SphericalDepthImage sph_Image = SphericalDepthImage(point_cloud_with_normal,params);
    sph_Image.initializeIndexImage();
    sph_Image.executeOperations();

    // std::shared_ptr<std::vector<Matchable>>  matchablesPtr( new std::vector<Matchable>());
    //std::vector<Matchable> matchables = sph_Image.clusterizeCloud(); no more 
    
    MatchablePtrVecPtr matchablePtrVecPtr = std::make_shared< std::vector< MatchablePtr>>();
    sph_Image.clusterizeCloud( matchablePtrVecPtr);
 

    std::cout << " Number of matchables : "<<matchablePtrVecPtr->size() << "\n"; 

    (*_dest).reserve( matchablePtrVecPtr->size() );
    for ( auto  m : *matchablePtrVecPtr){

      CustomMatchablef customMatchable(
          Matchablef::Type::Line,
          m->get_p_m(),
          m->get_R_m()
          );

      (*_dest).push_back( customMatchable  );
    }
  }

   bool CustomMeasurementAdaptor::setMeasurement(BaseSensorMessagePtr measurement_){
     BaseType::setMeasurement( measurement_);
     return true;
   };
}


