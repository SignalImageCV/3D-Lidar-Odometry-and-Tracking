#include "../loam/Visualizer.hpp"

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;


void clusterizeFeatures( ViewerCanvasPtr canvas){

  string filename = "/home/dinies/temp/trial/tuttty.boss";
  DatasetManager dM( filename);

  SphericalDepthImage sph_Image;

  const sphericalImage_params params(
      40, //num_vertical_rings
      40, //num_points_ring
      0, //epsilon_times
      0, //epsilon_radius
      1, //depth_differential_threshold
      7,  //min_neighboors_for_normal
      5, //epsilon_c
      3, //epsilon_d
      0.1, //epsilon_n
      1, //epsilon_l
      1, //epsilon_dl
      1, //epsilon_p
      1 //epsilon_dp
  );


  while( ViewerCoreSharedQGL::isRunning()){

    PointNormalColor3fVectorCloud points =  dM.readMessageFromDataset();
    sph_Image = SphericalDepthImage(points,params);
    sph_Image.initializeIndexImage();
    IntegralImage integ_img =  sph_Image.collectNormals();
    //std::vector<Matchable> matchables = sph_Image.clusterizeCloud( integ_img);
    PointNormalColor3fVectorCloud resulting_cloud = sph_Image.getPointCloud();
    canvas->putPoints(resulting_cloud);
    canvas->flush();
    //cout<< "num of matchables " << matchables.size()<< "\n";
  }
}

int main( int argc, char** argv){

  ViewerCoreSharedQGL viewer(argc, argv);
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingCloud");
  std::thread processing_thread(clusterizeFeatures, canvas);
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}


