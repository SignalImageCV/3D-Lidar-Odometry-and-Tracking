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
      2, //epsilon_d
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


    PointNormalColor3fVectorCloud p2 = Visualizer::createPlane(
      Vector3f( -7.,7.,0.),Vector3f( 0.,0.,1.),
      Vector3f( 1.,1.,0.).normalized(), 4, 6, 0.5, 0.5); 

    cloud.insert(
      cloud.end(),
      std::make_move_iterator( p2.begin()),
      std::make_move_iterator( p2.end())
    );



    sph_Image = SphericalDepthImage(cloud,params);
    sph_Image.initializeIndexImage();
    IntegralImage integ_img =  sph_Image.collectNormals();
    
    vector< vector< int>> goodSeeds =  sph_Image.findGoodClusterSeeds();


    std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >  colors;
    const int num_colors = goodSeeds.size();
    colors.resize(num_colors);
    for(size_t i=0; i < colors.size(); ++i) {
      colors[i]= Vector3f(
          0.f* float(i)/num_colors,
          0.f* (1 - float(i)/num_colors),
          255.f* (1 - float(i)/num_colors));
    }
    vector<vector<list<DataPoint>>> curr_index_image =  sph_Image.getIndexImage();

    while( ViewerCoreSharedQGL::isRunning()){
        canvas->setPointSize(3);
        int color_index = 0;
        for ( auto & seed: goodSeeds){
          const int seed_row = seed[0];
          const int seed_col = seed[1] ;
          const int seed_list_pos = seed[2];
          auto curr_seed = std::next( curr_index_image[seed_row][seed_col].begin(), seed_list_pos);

          vector<int> boundaries = curr_seed->getBoundaries();

          const int row_min = boundaries[0];
          const int row_max = boundaries[1];
          const int col_min = boundaries[2];
          const int col_max = boundaries[3];
   
          PointNormalColor3fVectorCloud seedPoints =
            sph_Image.fetchPointsInBoundaries( row_min, row_max, col_min, col_max);


          for( auto & p: seedPoints){
            p.color() = colors[color_index];
          }
          canvas->putPoints(seedPoints);
          ++color_index;
        }
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

