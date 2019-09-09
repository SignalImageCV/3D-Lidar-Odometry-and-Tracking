#include "../loam/Visualizer.hpp"
#include <srrg_messages_ros/message_handlers/message_rosbag_source.h>
#include <srrg_system_utils/parse_command_line.h>


using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_qgl_viewport;
using namespace Loam;


const char* banner[] = {
    "dynamic executor",
      0
};


void visualizeClouder( ViewerCanvasPtr canvas, string filename){

  MessageROSBagSource m_source;
  m_source.param_topics.value().push_back("/kitti/velo/pointcloud");
  m_source.open(filename);
  
  const sphericalImage_params params(
    64, //num_vertical_rings
    768, //num_points_ring
    7, //epsilon_times
    0.15, //epsilon_radius
    1, //depth_differential_threshold
    4,  //min_neighboors_for_normal
    8, //epsilon_c
    0.1, //epsilon_d
    0.02, //epsilon_n
    1, //epsilon_l
    1, //epsilon_dl
    1, //epsilon_p
    1 //epsilon_dp
  );
 
  SphericalDepthImage sph_Image;


  BaseSensorMessagePtr msg;
  Point3fVectorCloud current_point_cloud;
  while (msg=m_source.getMessage()){
    PointCloud2MessagePtr cloud = std::dynamic_pointer_cast<PointCloud2Message>(msg);
    if(cloud){
      cloud->getPointCloud(current_point_cloud);
    }


  PointNormalColor3fVectorCloud cloud_new;
  cloud_new.resize( current_point_cloud.size());
  current_point_cloud.copyFieldTo<0,0,PointNormalColor3fVectorCloud>(cloud_new);

  for ( auto & p : cloud_new){
    p.color() = ColorPalette::color3fDarkCyan();
  }

  sph_Image = SphericalDepthImage(cloud_new,params);
  sph_Image.initializeIndexImage();
  sph_Image.removeFlatSurfaces();
  sph_Image.executeOperations();

  const PointNormalColor3fVectorCloud cloud_final = sph_Image.getPointCloud() ;
  
  canvas->pushPointSize();
  canvas->setPointSize(1.5f);
  canvas->putPoints( cloud_final);
  canvas->popAttribute();
  canvas->flush();
  }
}

int main( int argc, char** argv){
  ParseCommandLine cmd_line(argv,banner);
  ArgumentString  dataset (&cmd_line, "d", "dataset", "path to dataset" , "");
  cmd_line.parse();

  messages_registerTypes();
  srrgInit( argc, argv, "hi");
 


  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas1 = viewer.getCanvas("cloud");

  std::thread processing_thread(
      visualizeClouder,canvas1, dataset.value());

  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}
