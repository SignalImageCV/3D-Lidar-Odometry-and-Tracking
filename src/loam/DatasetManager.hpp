#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>

namespace Loam {

  class DatasetManager{

  public:
    DatasetManager();
    void readDataset();

  };
}

