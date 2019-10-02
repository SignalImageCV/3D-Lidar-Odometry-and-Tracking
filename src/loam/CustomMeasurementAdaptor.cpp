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

    m_sph_ImagePtr = std::make_shared<SphericalDepthImage>( SphericalDepthImage(point_cloud_with_normal,params));

    m_sph_ImagePtr->initializeIndexImage();
    m_sph_ImagePtr->executeOperations();

    m_sph_ImagePtr->clusterizeCloud( m_matchablePtrVecPtr);


   // std::cout << " Number of matchables : "<<matchablePtrVecPtr->size() << "\n"; 

    (*_dest).reserve( m_matchablePtrVecPtr->size() );
    for ( auto  m : *m_matchablePtrVecPtr){
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

   void CustomMeasurementAdaptor::setMyParams( const sphericalImage_params & t_params){

     param_num_vertical_rings.setValue(t_params.num_vertical_rings);
     param_num_points_ring.setValue(t_params.num_points_ring);
     param_epsilon_times.setValue(t_params.epsilon_times);
     param_epsilon_radius.setValue(t_params.epsilon_radius);
     param_depth_differential_threshold.setValue(t_params.depth_differential_threshold);
     param_min_neighboors_for_normal.setValue(t_params.min_neighboors_for_normal);
     param_epsilon_c.setValue(t_params.epsilon_c);
     param_epsilon_d.setValue(t_params.epsilon_d);
     param_epsilon_n.setValue(t_params.epsilon_n);
     param_epsilon_l.setValue(t_params.epsilon_l);
     param_epsilon_dl.setValue(t_params.epsilon_dl);
     param_epsilon_p.setValue(t_params.epsilon_p);
     param_epsilon_dp.setValue(t_params.epsilon_dp);
   }

   PointNormalColor3fVectorCloud CustomMeasurementAdaptor::drawClusters(){
     PointNormalColor3fVectorCloud clusters_cloud;
     if ( m_sph_ImagePtr){
       clusters_cloud = m_sph_ImagePtr->drawClusters3D();
     }
     return clusters_cloud;
   }

   PointNormalColor3fVectorCloud CustomMeasurementAdaptor::drawMatchables(){

     PointNormalColor3fVectorCloud features_cloud;
     if ( m_sph_ImagePtr and m_matchablePtrVecPtr->size()>0){

       features_cloud.reserve( m_sph_ImagePtr->getPointCloud().size());

       PointNormalColor3fVectorCloud curr_drawing_points;
       const float length= 5;
       const float precision = 0.5;

       const int num_colors = m_matchablePtrVecPtr->size();
       int color_counter = 0;
       Vector3f currentColor = Vector3f::Zero();
       for ( auto m : *m_matchablePtrVecPtr){

         int color_index = color_counter* 256.f / num_colors;  
         currentColor = Vector3f(
             turbo_srgb_floats[color_index][0],
             turbo_srgb_floats[color_index][1],
             turbo_srgb_floats[color_index][2]);

         ++color_counter;

         curr_drawing_points = m->drawMatchable(length,  precision, currentColor);

         features_cloud.insert(
             features_cloud.end(),
             std::make_move_iterator( curr_drawing_points.begin()),
             std::make_move_iterator( curr_drawing_points.end())
             );
       }
     }
     return  features_cloud;
   }

   void CustomMeasurementAdaptor::reset(){
    BaseType::reset();
    m_matchablePtrVecPtr = std::make_shared< std::vector< MatchablePtr>>();
   };
}


