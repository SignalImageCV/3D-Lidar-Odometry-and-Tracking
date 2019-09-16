#include "../loam/Visualizer.hpp"
#include <srrg_system_utils/parse_command_line.h>

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;

const char* banner[] = {
    "dynamic executor",
      0
};

int main( int argc, char** argv){

  ParseCommandLine cmd_line(argv,banner);
  ArgumentString  dataset (&cmd_line, "d", "dataset", "path to dataset" , "");
  cmd_line.parse();

  messages_registerTypes();
  srrgInit( argc, argv, "hi");
 

  const sphericalImage_params params(
      64, //num_vertical_rings
      768, //num_points_ring
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

  DatasetManager dM( dataset.value());
  RGBImage index_img; 
  RGBImage normals_img;

  cv::namedWindow("IndexImage");
  cv::moveWindow("IndexImage", 20, 40);

  cv::namedWindow("NormalsImage");
  cv::moveWindow("NormalsImage", 20, 740);

  RGBImage index_img_resized; 
  RGBImage normals_img_resized;
  const float horizontal_scale= 1;
  const float vertical_scale= 1;
  

  PointNormalColor3fVectorCloud current_point_cloud= dM.readMessageFromDataset();
  while( current_point_cloud.size()> 0){

    sph_Image= SphericalDepthImage(current_point_cloud,params);
    sph_Image.initializeIndexImage();
    sph_Image.collectNormals();
    sph_Image.removeFlatSurfaces();
    index_img = sph_Image.drawIndexImg(); 
    normals_img = sph_Image.drawNormalsImg();
    cv::resize( index_img, index_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);
    cv::resize( normals_img, normals_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);

    cv::imshow("IndexImage",index_img_resized);
    cv::imshow("NormalsImage",normals_img_resized);
    cv::waitKey(1);
   
    current_point_cloud = dM.readMessageFromDataset();
  }

  return 0;
}
