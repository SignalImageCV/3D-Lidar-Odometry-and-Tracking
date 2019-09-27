#pragma once

#include <srrg_config/configurable.h>
#include <srrg_config/property_configurable.h>
#include <srrg_slam_interfaces/measurement_adaptor.h>
#include <srrg_messages/instances.h>
#include <srrg_matchable/visual_matchable.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_pcl/point_unprojector_types.h>

#include "features/SphericalDepthImage.hpp"
#include "features/CustomMatchable.hpp"

namespace Loam{
  using namespace srrg2_core;

  class CustomMeasurementAdaptor: public srrg2_slam_interfaces::MeasurementAdaptor_<CustomMatchablefVector> { 
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = CustomMeasurementAdaptor;
    using BaseType = srrg2_slam_interfaces::MeasurementAdaptor_<CustomMatchablefVector>;
    using DestType = typename BaseType::DestType;

    PARAM(PropertyUnsignedInt, num_vertical_rings, "num_vertical_rings", 64, nullptr);
    PARAM(PropertyUnsignedInt, num_points_ring, "num_points_ring", 768, nullptr);
    PARAM(PropertyUnsignedInt, epsilon_times, "epsilon_times", 7, nullptr);
    PARAM(PropertyFloat, epsilon_radius, "epsilon_radius",0.15, nullptr);
    PARAM(PropertyFloat, depth_differential_threshold, "depth_differential_threshold",2.1, nullptr);
    PARAM(PropertyUnsignedInt, min_neighboors_for_normal, "min_neighboors_for_normal", 7, nullptr);
    PARAM(PropertyUnsignedInt, epsilon_c, "epsilon_c", 20, nullptr);
    PARAM(PropertyFloat, epsilon_d, "epsilon_d",1.5, nullptr);
    PARAM(PropertyFloat, epsilon_n, "epsilon_n",0.3, nullptr);
    PARAM(PropertyFloat, epsilon_l, "epsilon_l",0.1, nullptr);
    PARAM(PropertyFloat, epsilon_dl, "epsilon_dl",0.1, nullptr);
    PARAM(PropertyFloat, epsilon_p, "epsilon_p",0.4, nullptr);
    PARAM(PropertyFloat, epsilon_dp, "epsilon_dp",0.5, nullptr);

    void compute() override;
    bool setMeasurement(BaseSensorMessagePtr measurement_) override;

    
  };

  using CustomMeasurementAdaptorPtr = std::shared_ptr<CustomMeasurementAdaptor>;

}
