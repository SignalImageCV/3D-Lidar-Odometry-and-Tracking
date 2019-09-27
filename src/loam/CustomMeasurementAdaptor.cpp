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

    MatchablePtrVecPtr matchablePtrVecPtr = std::make_shared< std::vector< MatchablePtr>>();
    sph_Image.clusterizeCloud( matchablePtrVecPtr);

    //maybe it shold be nice to memorize this pointer matchablePtrVecPtr then add a function
    //for drawing clusters as is done in the drawMatchablesRealData test
    //that can be called from the outside, namely from the aligner for example
    //or every oher class or test, this way I can understand the situation of the
    //features at each iteration ( issue, the feature matcher does not match the features)
    //maybe they are too much rototranslated between pointclouds.
 

   // std::cout << " Number of matchables : "<<matchablePtrVecPtr->size() << "\n"; 

    (*_dest).reserve( matchablePtrVecPtr->size() );
    for ( auto  m : *matchablePtrVecPtr){
      string className = m->get_ClassName();
      Matchablef::Type type;
      if ( className== "Line" ){
        type=Matchablef::Type::Line; 
      }
      else if( className== "Plane" ){
        type=Matchablef::Type::Plane; 
      }
      else {
        type=Matchablef::Type::Point; 
      }

      CustomMatchablef customMatchable(
          type,
          m->get_p_m(),
          m->get_R_m()
          );

   //   std::cout << "MyMatchable origin : "<<m->get_p_m().transpose() << "\n"; 
   //   std::cout << "MyMatchable direction : "<<m->get_R_m().col(0).transpose() << "\n"; 


      (*_dest).push_back( customMatchable  );
    }
  }

   bool CustomMeasurementAdaptor::setMeasurement(BaseSensorMessagePtr measurement_){
     BaseType::setMeasurement( measurement_);
     return true;
   };
}


