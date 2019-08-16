#include "../loam/Visualizer.hpp"

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;


int main( int argc, char** argv){

  const sphericalImage_params params(
    60, //num_vertical_rings
    200, //num_points_ring
    6, //epsilon_times
    0.15, //epsilon_radius
    0.1, //depth_differential_threshold
    6,  //min_neighboors_for_normal
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

  PointNormalColor3fVectorCloud noise;
  noise.resize(7);
  Vector3f coords_p_1 = Vector3f(-4., 4., 4. );
  Vector3f coords_p_2 = Vector3f(3., -3., 8. );
  Vector3f coords_p_3 = Vector3f(-1., 1., 5. );
  Vector3f coords_p_4 = Vector3f(7., -1., 2. );
  Vector3f coords_p_5 = Vector3f(-7., -7., 3. );
  Vector3f coords_p_6 = Vector3f(-1., -7., 4. );
  Vector3f coords_p_7 = Vector3f(3., -5., 0. );
  noise[0].coordinates() = coords_p_1;
  noise[1].coordinates() = coords_p_2;
  noise[2].coordinates() = coords_p_3;
  noise[3].coordinates() = coords_p_4;
  noise[4].coordinates() = coords_p_5;
  noise[5].coordinates() = coords_p_6;
  noise[6].coordinates() = coords_p_7;


  PointNormalColor3fVectorCloud l1 = Visualizer::createLine(
    Vector3f( 2.,3.,1.), Vector3f( 0.,0.,1.), 8, 0.1);
  PointNormalColor3fVectorCloud l2 = Visualizer::createLine(
    Vector3f( -4.,-2.,-2.), Vector3f( 0.,0.,-1.), 10, 0.2);
  PointNormalColor3fVectorCloud p1 = Visualizer::createPlane(
    Vector3f( 5.,5.,0.),Vector3f( 0.,0.,1.),
    Vector3f( 1.,-1.,0.).normalized(), 4, 8, 0.5, 0.5);
  

  cloud.insert(
  cloud.end(),
    std::make_move_iterator( l1.begin()),
    std::make_move_iterator( l1.end())
  );

  cloud.insert(
    cloud.end(),
    std::make_move_iterator( l2.begin()),
    std::make_move_iterator( l2.end())
  );

  cloud.insert(
    cloud.end(),
    std::make_move_iterator( p1.begin()),
    std::make_move_iterator( p1.end())
  );

  sph_Image = SphericalDepthImage(cloud,params);
  sph_Image.initializeIndexImage();
  std::vector<Matchable> matchables = sph_Image.clusterizeCloud();
    
  RGBImage index_img; 
  RGBImage normals_img;
  RGBImage path_img; 
  RGBImage blurred_normals_img;


  cv::namedWindow("IndexImage");
  cv::moveWindow("IndexImage", 20, 240);
  cv::namedWindow("NormalsImage");
  cv::moveWindow("NormalsImage", 20, 440);
  cv::namedWindow("PathImage");
  cv::moveWindow("PathImage", 20, 640);
  cv::namedWindow("BlurredNormalsImage");
  cv::moveWindow("BlurredNormalsImage", 20, 840);
  index_img = sph_Image.drawIndexImg(); 
  normals_img = sph_Image.drawNormalsImg();
  vector<RGBImage> clusterer_imgs = sph_Image.drawImgsClusterer();
  path_img = clusterer_imgs[0];
  blurred_normals_img = clusterer_imgs[1];

  int counter = 0;

  while( counter < 10000){
    ++counter;
    cv::imshow("IndexImage",index_img);
    cv::imshow("NormalsImage",normals_img);
    cv::imshow("PathImage",path_img);
    cv::imshow("BlurredNormalsImage",blurred_normals_img);
    cv::waitKey(1);
  }

  return 0;
}
