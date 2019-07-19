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
  RGBImage m_drawing_index_img;
  RGBImage m_drawing_normals;

  m_drawing_index_img.create( params.num_vertical_rings, params.num_points_ring);
  m_drawing_index_img= cv::Vec3b(253, 246, 227);
  cv::namedWindow("IndexImage");
  cv::moveWindow("IndexImage", 20, 40);

  m_drawing_normals.create( params.num_vertical_rings, params.num_points_ring);
  m_drawing_normals= cv::Vec3b(253, 246, 227);
  cv::namedWindow("NormalsImage");
  cv::moveWindow("NormalsImage", 20, 240);



  PointNormalColor3fVectorCloud current_point_cloud= dM.readMessageFromDataset();
  while( current_point_cloud.size()> 0){

 
    sph_Image= SphericalDepthImage(current_point_cloud,params);
    //sph_Image.showImages(1);

    sph_Image.removeFlatSurfaces();
    sph_Image.collectNormals();
    cout <<"ARRIVEDDDDD\n";
    m_drawing_index_img = sph_Image.drawIndexImg(); 
    m_drawing_normals = sph_Image.drawNormalsImg();
    cv::imshow("IndexImage",m_drawing_index_img);
    cv::imshow("NormalsImage",m_drawing_normals);
    cv::waitKey(1);
    //sph_Image.drawImages(1);

    current_point_cloud = dM.readMessageFromDataset();
  }

  return 0;
}
