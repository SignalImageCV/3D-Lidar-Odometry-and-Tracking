#include "../loam/Visualizer.hpp"

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;

  void drawingClusterBoundaries( ViewerCanvasPtr canvas){
    SphericalDepthImage sph_Image;
    PointNormalColor3fVectorCloud cloud;

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

    PointNormalColor3f min_elev;
    min_elev.coordinates() = Vector3f( 5,5,10);
    cloud.push_back( min_elev);
    PointNormalColor3f max_elev;
    max_elev.coordinates() = Vector3f( 5,5,-10);
    cloud.push_back( max_elev);

    PointNormalColor3fVectorCloud p1 = Visualizer::createPlane(
      Vector3f( 5.,5.,0.),Vector3f( 0.,0.,1.),
      Vector3f( 1.,-1.,0.).normalized(), 4, 6, 0.5, 0.5); 

    cloud.insert(
      cloud.end(),
      std::make_move_iterator( p1.begin()),
      std::make_move_iterator( p1.end())
    );
    sph_Image = SphericalDepthImage(cloud,params);
    sph_Image.initializeIndexImage();
    IntegralImage integ_img =  sph_Image.collectNormals();

    int k = 4;
    vector<int> indexes=  sph_Image.mapCartesianCoordsInIndexImage(cloud[k].coordinates());
    sph_Image.discoverClusterBoundaryIndexes(indexes[0], indexes[1],0);

    PointNormalColor3fVectorCloud current_cloud = sph_Image.getPointCloud();
    PointNormalColor3fVectorCloud drawingPoints;
    drawingPoints.reserve( p1.size());

    for (unsigned int row =0; row <sph_Image.getIndexImage().size() ; ++row){
      for (unsigned int col=0; col <sph_Image.getIndexImage()[0].size(); ++col){
        for ( auto entry : sph_Image.getIndexImage()[row][col]){

          PointNormalColor3f curr_point = current_cloud[entry.getIndexContainer()];
          if ( entry.getIsClustered()){
            curr_point.color() = ColorPalette::color3fOrange();
          }
          else{
            curr_point.color() = ColorPalette::color3fDarkBlue();
          }
          drawingPoints.push_back( curr_point);
        }
      }
    }

    while( ViewerCoreSharedQGL::isRunning()){
        canvas->setPointSize(3);
        canvas->putPoints( drawingPoints);
        canvas->flush();
    }
  }
          

 

int main( int argc, char** argv){

  ViewerCoreSharedQGL viewer(argc, argv);
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingclusterBoundaries");
  std::thread processing_thread( drawingClusterBoundaries, canvas);
  viewer.startViewerServer();
  processing_thread.join();

  return 0;
}

