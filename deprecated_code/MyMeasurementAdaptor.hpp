#pragma once

#include <srrg_config/configurable.h>
#include <srrg_config/property_configurable.h>
#include <srrg_slam_interfaces/measurement_adaptor.h>
#include <srrg_messages/instances.h>
#include <srrg_matchable/visual_matchable.h>

#include <srrg_messages/messages/image_message.h>
#include <srrg_pcl/point_unprojector_types.h>
#include <srrg_slam_interfaces/measurement_adaptor.h>

#include "./features/SphericalDepthImage.hpp"
#include "./MyToyConfigurable.hpp"

//todo remove this stub class: used to understand the srrg2 pipeline
namespace Loam{
  using namespace srrg2_core;

  class MyMeasurementAdaptor: public srrg2_slam_interfaces::MeasurementAdaptor_<VisualMatchablefVector> { 
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = MyMeasurementAdaptor;
    using BaseType = srrg2_slam_interfaces::MeasurementAdaptor_<VisualMatchablefVector>;
    using DestType = typename BaseType::DestType;

    PARAM(PropertyConfigurable_<MyToyConfigurable>,
        toy,
        "num_vertical_rings",
        MyToyConfigurablePtr( new MyToyConfigurable),
        nullptr);

    PARAM(PropertyUnsignedInt, num_vertical_rings, "num_vertical_rings", 64, nullptr);
    PARAM(PropertyUnsignedInt, num_points_ring, "num_points_ring", 768, nullptr);
    PARAM(PropertyUnsignedInt, epsilon_times, "epsilon_times", 7, nullptr);
    PARAM(PropertyFloat, epsilon_radius, "epsilon_radius",0.15, nullptr);
    PARAM(PropertyFloat, depth_differential_threshold, "depth_differential_threshold",2.1, nullptr);
    PARAM(PropertyUnsignedInt, min_neighboors_for_normal, "min_neighboors_for_normal", 7, nullptr);
    PARAM(PropertyUnsignedInt, epsilon_c, "epsilon_c", 8, nullptr);
    PARAM(PropertyFloat, epsilon_d, "epsilon_d",1.5, nullptr);
    PARAM(PropertyFloat, epsilon_n, "epsilon_n",0.3, nullptr);
    PARAM(PropertyFloat, epsilon_l, "epsilon_l",1, nullptr);
    PARAM(PropertyFloat, epsilon_dl, "epsilon_dl",1, nullptr);
    PARAM(PropertyFloat, epsilon_p, "epsilon_p",1, nullptr);
    PARAM(PropertyFloat, epsilon_dp, "epsilon_dp",1, nullptr);

    void compute() override;
    bool setMeasurement(BaseSensorMessagePtr measurement_) override;

    
  };

  using MyMeasurementAdaptorPtr = std::shared_ptr<MyMeasurementAdaptor>;

} 
