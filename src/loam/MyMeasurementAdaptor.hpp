#pragma once

#include <srrg_config/configurable.h>
#include <srrg_config/property_configurable.h>
#include <srrg_slam_interfaces/measurement_adaptor.h>
#include <srrg_messages/instances.h>
#include <srrg_matchable/visual_matchable.h>



namespace Loam{
  using namespace srrg2_core;

  class MyMeasurementAdaptor

    : public srrg2_slam_interfaces::MeasurementAdaptor_<VisualMatchableVector> { 
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = MyMeasurementAdaptor;
    using BaseType = srrg2_slam_interfaces::MeasurementAdaptor_<VisualMatchableVector>>;
    using typename BaseType::DestType;


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

  protected:
    //! @brief shitty aux function to process messages
    void _processLaserMessage(LaserMessagePtr message_);

    //! @brief laser scan fields - mandatory
    std::vector<float>* _ranges = nullptr;
  };

  using MyMeasurementAdaptorPtr = std::shared_ptr<MyMeasurementAdaptor>;

} 
