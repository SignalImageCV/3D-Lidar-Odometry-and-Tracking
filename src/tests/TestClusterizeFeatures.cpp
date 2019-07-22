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

    vector< vector< int>> goodSeeds =  sph_Image.findGoodClusterSeeds();

    std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >  colors;
    const int num_colors = goodSeeds.size();
    colors.resize(num_colors);
    for(size_t i=0; i < colors.size(); ++i) {
      colors[i]= Vector3f(
          227.f* float(i)/num_colors,
          246.f* (1 - float(i)/num_colors),
          253.f* float(i)/num_colors);
    }

    vector<vector<list<DataPoint>>> curr_index_image =  sph_Image.getIndexImage();

    int color_index= 0;
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
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingCloud");
  std::thread processing_thread(clusterizeFeatures, canvas);
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}


