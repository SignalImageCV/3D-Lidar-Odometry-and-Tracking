#include "../loam/Visualizer.hpp"

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;


int main( int argc, char** argv){

  string filename = "/home/dinies/temp/trial/tuttty.boss";

  const sphericalImage_params params(
      60, //num_vertical_rings
      2000, //num_points_ring
      10, //epsilon_times
      0.15, //epsilon_radius
      0.1, //depth_differential_threshold
      8,  //min_neighboors_for_normal
      5, //epsilon_c
      0.1, //epsilon_d
      0.02, //epsilon_n
      1, //epsilon_l
      1, //epsilon_dl
      1, //epsilon_p
      1 //epsilon_dp
    );
        
  SphericalDepthImage sph_Image;
  DatasetManager dM( filename);
  RGBImage index_img; 
  RGBImage normals_img;
  RGBImage path_img;

  cv::namedWindow("IndexImage");
  cv::moveWindow("IndexImage", 10, 40);

  cv::namedWindow("NormalsImage");
  cv::moveWindow("NormalsImage", 10, 240);

  cv::namedWindow("PathImage");
  cv::moveWindow("PathImage", 10, 440);



  const int iterations = 10000;
  int curr_iteration = 0;
  PointNormalColor3fVectorCloud current_point_cloud= dM.readMessageFromDataset();

 
  sph_Image= SphericalDepthImage(current_point_cloud,params);
  sph_Image.removeFlatSurfaces();
  sph_Image.collectNormals();
  index_img = sph_Image.drawIndexImg(); 
  normals_img = sph_Image.drawNormalsImg();
  Clusterer c= Clusterer( sph_Image.getPointCloud(), sph_Image.getIndexImage(), params);
  path_img = c.drawPathImg();

  while( curr_iteration < iterations){
    cv::imshow("IndexImage",index_img);
    cv::imshow("NormalsImage",normals_img);
    cv::imshow("PathImage",path_img);
    cv::waitKey(1);
    ++curr_iteration;
  }
  return 0;
}
