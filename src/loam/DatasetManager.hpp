#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_messages_ros/message_handlers/message_rosbag_source.h>

namespace Loam {
  using namespace std;
  using namespace srrg2_core;
  using namespace srrg2_core_ros;


  class DatasetManager{

  private:
    MessageROSBagSource m_source;
    int m_current_index;

  public:
    DatasetManager( string filename);

    PointNormalColor3fVectorCloud  readMessageFromDataset();
  };
}

