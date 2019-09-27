#include "loam/features/SphericalDepthImage.hpp"
#include "loam/Visualizer.hpp"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;

int main( int argc, char** argv){


  const sphericalImage_params params(
    64, //num_vertical_rings
    768, //num_points_ring
    7, //epsilon_times
    0.15, //epsilon_radius
    2.1, //depth_differential_threshold
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
  min_elev.coordinates() = Vector3f( 2,7,10);
  cloud.push_back( min_elev);
  PointNormalColor3f max_elev;
  max_elev.coordinates() = Vector3f( 2,7,-10);
  cloud.push_back( max_elev);

  //PointNormalColor3fVectorCloud l1 = Drawer::createLine(
  // Vector3f( 5.,5.,-5.),Vector3f( 0.,0.,1.),
  //  10, 0.25);

  PointNormalColor3fVectorCloud p1 = Drawer::createPlane(
    Vector3f( 35.,35.,0.),Vector3f( 0.,0.,1.),
    Vector3f( -1.,1.,0.).normalized(), 18, 14, 0.25, 0.25);

  PointNormalColor3fVectorCloud p2 = Drawer::createPlane(
    Vector3f( 0.,35.,0.),Vector3f( 0.,0.,1.),
    Vector3f( 1.,0.,0.), 18, 14, 0.25, 0.25);

  cloud.insert(
    cloud.end(),
    std::make_move_iterator( p1.begin()),
    std::make_move_iterator( p1.end())
  );


  cloud.insert(
    cloud.end(),
    std::make_move_iterator( p2.begin()),
    std::make_move_iterator( p2.end())
  );

  sph_Image = SphericalDepthImage(cloud,params);
  sph_Image.initializeIndexImage();

  const PointNormalColor3fVectorCloud cloud_before_cleaning = sph_Image.getPointCloud();

  sph_Image.removeFlatSurfaces();

  const PointNormalColor3fVectorCloud cloud_after_flatSurfRemoval=  sph_Image.getPointCloud();

  cout << "number of points after horizontal surf removed: " << cloud_after_flatSurfRemoval.size() <<"\n";

  sph_Image.discoverNormalsBoundaryIndexes();

  const PointNormalColor3fVectorCloud cloud_after_indexesDiscovery=  sph_Image.getPointCloud();

  vector<vector< DataPoint >> index_image =  sph_Image.getIndexImage();

//  for (unsigned int row =0; row <index_image.size() ; ++row){
//    for (unsigned int col=0; col <index_image[0].size(); ++col){
//      if(index_image[row][col].getIndexContainer() != -1){
//        vector< int>  boundaries = index_image[row][col].getBoundaries();
//        cout<< "coords : "<< row<< ", " << col<< " \n";
//        cout<< "boundary : "<< boundaries[0] << ", " << boundaries[1] << ", " << boundaries[2] << ", " << boundaries[3] << "\n "; 
//      }
//    }
//  }

  sph_Image.removePointsWithoutNormal();
  sph_Image.computePointNormals();

  const PointNormalColor3fVectorCloud cloud_after_NormalComputation=  sph_Image.getPointCloud();
  cout << "number of points after normal computation : " << cloud_after_NormalComputation.size() <<"\n";

 // for ( auto & p : cloud_after_NormalComputation){
//    cout << "normal : "<< p.normal() << "\n";
//  }
    

  const float points_size = 2.0;

  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
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
