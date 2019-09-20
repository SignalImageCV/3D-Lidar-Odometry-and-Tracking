#include "../loam/Visualizer.hpp"
#include <srrg_system_utils/parse_command_line.h>

using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_qgl_viewport;
using namespace Loam;

const char* banner[] = {
    "dynamic executor",
      0
};


int main( int argc, char** argv){

  messages_registerTypes();
  srrgInit( argc, argv, "hi");



  ParseCommandLine cmd_line(argv,banner);
  ArgumentString  dataset (&cmd_line, "d", "dataset", "path to dataset" , "");
  cmd_line.parse();

  const sphericalImage_params params(
    64, //num_vertical_rings
    768, //num_points_ring
    7, //epsilon_times
    0.15, //epsilon_radius
    2.1, //depth_differential_threshold
    7,  //min_neighboors_for_normal
    8, //epsilon_c
    1.5, //epsilon_d
    0.3, //epsilon_n
    1, //epsilon_l
    1, //epsilon_dl
    1, //epsilon_p
    1 //epsilon_dp
    );
  

  DatasetManager dM( dataset.value());
  PointNormalColor3fVectorCloud cloud = dM.readMessageFromDataset();

  SphericalDepthImage sph_Image;
  sph_Image = SphericalDepthImage(cloud,params);
  sph_Image.initializeIndexImage();
  sph_Image.executeOperations();

  MatchablePtrVecPtr matchablePtrVecPtr = std::make_shared< std::vector< MatchablePtr>>();
  sph_Image.clusterizeCloud( matchablePtrVecPtr);

  PointNormalColor3fVectorCloud features_cloud;
  features_cloud.reserve( cloud.size());

  PointNormalColor3fVectorCloud curr_drawing_points;
  const float length= 10;
  const float precision = 0.1;


  for ( auto m : *matchablePtrVecPtr){
    curr_drawing_points = m->drawMatchable(length,  precision );

    features_cloud.insert(
      features_cloud.end(),
      std::make_move_iterator( curr_drawing_points.begin()),
      std::make_move_iterator( curr_drawing_points.end())
    );
  }
 


  messages_registerTypes();
  srrgInit( argc, argv, "hi");

  const float points_size = 2.0;
  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas1 = viewer.getCanvas("points");
  std::thread processing_thread1( Visualizer::visualizeCloud, canvas1, cloud, points_size);

  ViewerCanvasPtr canvas2 = viewer.getCanvas("matchables");
  std::thread processing_thread2( Visualizer::visualizeCloud, canvas2, features_cloud, points_size);

  viewer.startViewerServer();
  processing_thread1.join();
  processing_thread2.join();
 
  return 0;
}




