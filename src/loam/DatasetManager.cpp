#include "DatasetManager.hpp"

namespace Loam {
  DatasetManager::DatasetManager(){};

  void DatasetManager::readDataset(){
    rosbag::Bag bag;
    bag.open("../datasets/nsh_undulating_motion.bag");  // BagMode is Read by default

    for(rosbag::MessageInstance const m: rosbag::View(bag)) {
      std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
      if (i != NULL)
        std::cout << i->data << std::endl;
    }
    bag.close();
  }

}
