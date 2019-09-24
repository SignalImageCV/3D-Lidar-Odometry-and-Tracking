#pragma once
#include <srrg_matchable/matchable.h>
#include <srrg_pcl/instances.h>
#include <srrg_slam_interfaces/measurement_adaptor.h>

#include "MyMeasurementAdaptor.hpp"
#include "./features/CustomMatchable.hpp"
#include "./CustomMeasurementAdaptor.hpp"
#include "./matcher/CorrespondenceFinderMatchablesKDtree.hpp"


namespace Loam {
  void loam_registerTypes();
}