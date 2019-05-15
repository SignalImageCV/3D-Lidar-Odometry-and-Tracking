#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>


int main( int argc, char** argv){

  rosbag::Bag bag;
  bag.open("/home/dinies/Downloads/nsh_staircase.bag");

  for(rosbag::MessageInstance const m: rosbag::View(bag)){
    std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
    if (i != NULL)
      std::cout << i->data << std::endl;
  }

  bag.close();
}



