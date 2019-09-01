#include "../loam/Visualizer.hpp"

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;

int main( int argc, char** argv){


  const sphericalImage_params params(
    60, //num_vertical_rings
    2000, //num_points_ring
    7, //epsilon_times
    0.15, //epsilon_radius
    0.1, //depth_differential_threshold
    7,  //min_neighboors_for_normal
    8, //epsilon_c
    0.1, //epsilon_d
    0.02, //epsilon_n
    1, //epsilon_l
    1, //epsilon_dl
    1, //epsilon_p
    1 //epsilon_dp
  );
  
  SphericalDepthImage sph_Image;
  PointNormalColor3fVectorCloud cloud;


  PointNormalColor3f min_elev;
  min_elev.coordinates() = Vector3f( 5,5,10);
  cloud.push_back( min_elev);
  PointNormalColor3f max_elev;
  max_elev.coordinates() = Vector3f( 5,5,-10);
  cloud.push_back( max_elev);

  PointNormalColor3fVectorCloud p1 = Visualizer::createPlane(
    Vector3f( 5.,5.,0.),Vector3f( 0.,0.,1.),
    Vector3f( 1.,0.,0.), 8, 8, 0.25, 0.25);

  //PointNormalColor3fVectorCloud p1 = Visualizer::createLine(
  //  Vector3f( 5.,5.,-5.),Vector3f( 0.,0.,1.),
  //  10, 0.25);


  cloud.insert(
    cloud.end(),
    std::make_move_iterator( p1.begin()),
    std::make_move_iterator( p1.end())
  );

  sph_Image = SphericalDepthImage(cloud,params);
  sph_Image.initializeIndexImage();

  const PointNormalColor3fVectorCloud cloud_before_cleaning = sph_Image.getPointCloud();

  sph_Image.removeFlatSurfaces();

  const PointNormalColor3fVectorCloud cloud_after_flatSurfRemoval=  sph_Image.getPointCloud();

  sph_Image.collectNormals();

  const PointNormalColor3fVectorCloud cloud_after_NormalComputation=  sph_Image.getPointCloud();


  const float points_size = 2.0;

  ViewerCoreSharedQGL viewer(argc, argv);
  ViewerCanvasPtr canvas1 = viewer.getCanvas("drawingPlaneBeforeCleaning");
  std::thread processing_thread1( Visualizer::visualizeCloud, canvas1, cloud_before_cleaning, points_size);

  ViewerCanvasPtr canvas2 = viewer.getCanvas("drawingPlaneAfterFlatSurfRemoval");
  std::thread processing_thread2( Visualizer::visualizeCloud, canvas2,cloud_after_flatSurfRemoval, points_size);

  ViewerCanvasPtr canvas3 = viewer.getCanvas("drawingPlaneAfterNormalComp");
  std::thread processing_thread3( Visualizer::visualizeCloud, canvas3,cloud_after_NormalComputation, points_size);



  viewer.startViewerServer();

  processing_thread1.join();
  processing_thread2.join();
  processing_thread3.join();

  return 0;
}
