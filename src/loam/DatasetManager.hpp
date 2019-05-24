#include <srrg_boss/id_context.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_boss/serializer.h>
#include <srrg_boss/deserializer.h>
#include <srrg_messages/instances.h>

#include "ScanPoint.hpp"

namespace Loam {
  using namespace std;
  using namespace srrg2_core;


  class DatasetManager{

  private:
    MessageFileSource m_source;
    int m_current_index;

  public:
    DatasetManager( string filename);
    vector<ScanPoint> readMessageFromDataset();
  };
}

