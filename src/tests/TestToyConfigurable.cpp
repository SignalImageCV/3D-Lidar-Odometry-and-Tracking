#include "../loam/MyMeasurementAdaptor.hpp"

using namespace srrg2_core;
using namespace Loam;

int main( int argc, char** argv){

  MyMeasurementAdaptorPtr  measurementAdaptor = MyMeasurementAdaptorPtr(new MyMeasurementAdaptor);
  measurementAdaptor->compute();
  return 0;
}

