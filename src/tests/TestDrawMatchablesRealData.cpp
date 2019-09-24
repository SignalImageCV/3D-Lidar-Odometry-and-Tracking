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
  ArgumentString  epsilon_l(&cmd_line, "l", "epsilon_l", "parameter that defines epsilon_l" , "");
  ArgumentString  epsilon_dl(&cmd_line, "dl", "epsilon_dl", "parameter that defines epsilon_dl" , "");
  ArgumentString  epsilon_p(&cmd_line, "p", "epsilon_p", "parameter that defines epsilon_p" , "");
  ArgumentString  epsilon_dp(&cmd_line, "dp", "epsilon_dp", "parameter that defines epsilon_dp" , "");
  cmd_line.parse();




  const sphericalImage_params params(
    64, //num_vertical_rings
    768, //num_points_ring
    7, //epsilon_times
    0.15, //epsilon_radius
    2.1, //depth_differential_threshold
    7,  //min_neighboors_for_normal
    20, //epsilon_c
    1.5, //epsilon_d
    0.3, //epsilon_n
    std::stof( epsilon_l.value()), //epsilon_l
    std::stof( epsilon_dl.value()), //epsilon_dl
    std::stof( epsilon_p.value()), //epsilon_p
    std::stof( epsilon_dp.value())//epsilon_dp
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
  const float length= 5;
  const float precision = 0.5;

  const int num_colors = matchablePtrVecPtr->size();
  int color_counter = 0;
  Vector3f currentColor = Vector3f::Zero();
  for ( auto m : *matchablePtrVecPtr){

   int color_index = color_counter* 256.f / num_colors;  
    currentColor = Vector3f(
      turbo_srgb_floats[color_index][0],
      turbo_srgb_floats[color_index][1],
      turbo_srgb_floats[color_index][2]);

    ++color_counter;

    curr_drawing_points = m->drawMatchable(length,  precision, currentColor);

    features_cloud.insert(
      features_cloud.end(),
      std::make_move_iterator( curr_drawing_points.begin()),
      std::make_move_iterator( curr_drawing_points.end())
    );
  }

  PointNormalColor3fVectorCloud clusters_points;

  Clusterer clusterer = Clusterer(sph_Image.getPointCloud(), sph_Image.getIndexImage() , params);
  vector<cluster> clusters = clusterer.findClusters();
  const int num_colors_clusters = clusters.size();
  int cluster_color_counter = 0;
  Vector3f currentClusterColor = Vector3f::Zero();


  for ( auto & c: clusters){
    PointNormalColor3fVectorCloud  curr_clusterPoints  = sph_Image.fetchPoints(c.indexes);
    int color_index = cluster_color_counter* 256.f / num_colors_clusters;  
    currentClusterColor = Vector3f(
      turbo_srgb_floats[color_index][0],
      turbo_srgb_floats[color_index][1],
      turbo_srgb_floats[color_index][2]);

    ++cluster_color_counter;

    for ( auto & p : curr_clusterPoints){
      p.color() = currentClusterColor;
    }

    clusters_points.insert(
      clusters_points.end(),
      std::make_move_iterator( curr_clusterPoints.begin()),
      std::make_move_iterator( curr_clusterPoints.end())
    );

  }
 

  messages_registerTypes();
  srrgInit( argc, argv, "hi");

  const float points_size = 2.0;
  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas1 = viewer.getCanvas("points");
  std::thread processing_thread1( Visualizer::visualizeCloud, canvas1, cloud, points_size);

  ViewerCanvasPtr canvas2 = viewer.getCanvas("clusters");
  std::thread processing_thread2( Visualizer::visualizeCloud, canvas2, clusters_points, points_size);

  ViewerCanvasPtr canvas3 = viewer.getCanvas("matchables");
  std::thread processing_thread3( Visualizer::visualizeCloud, canvas3, features_cloud, points_size);

  viewer.startViewerServer();
  processing_thread1.join();
  processing_thread2.join();
  processing_thread3.join();
  return 0;
}




