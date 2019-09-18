#include "../loam/CustomMeasurementAdaptor.hpp"
#include "../loam/instances.h"


using namespace srrg2_core;
using namespace Loam;

int main( int argc, char** argv){
  loam_registerTypes();

  CustomMeasurementAdaptorPtr measurementAdaptor = CustomMeasurementAdaptorPtr(new CustomMeasurementAdaptor);
  measurementAdaptor->compute();
  return 0;
}


/*
add_executable(test_customConfigurableExec
  TestCustomConfigurableExec.cpp
  )
target_link_libraries(test_toyConfigurable
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  srrg2_boss_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )
*/

