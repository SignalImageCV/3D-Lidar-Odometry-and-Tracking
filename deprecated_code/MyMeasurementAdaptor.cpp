#include "MyMeasurementAdaptor.hpp"

namespace Loam{

//todo remove this stub class: used to understand the srrg2 pipeline
  void MyMeasurementAdaptor::compute() {
    std::cout << " known number :" << this->param_toy.value()->spitOut() << "\n";

//    const sphericalImage_params params(
//      param_num_vertical_rings.value(),
//      param_num_points_ring,
//      param_epsilon_times,
//      param_epsilon_radius,
//      param_depth_differential_threshold,
//      param_min_neighboors_for_normal, 
//      param_epsilon_c,
//      param_epsilon_d,
//      param_epsilon_n,
//      param_epsilon_l,
//      param_epsilon_dl,
//      param_epsilon_p,
//      param_epsilon_dp,
//    );
    SphericalDepthImage sph_Image;
    PointNormalColor3fVectorCloud cloud;
  }

   bool MyMeasurementAdaptor::setMeasurement(BaseSensorMessagePtr measurement_){
     return true;
   };
}


