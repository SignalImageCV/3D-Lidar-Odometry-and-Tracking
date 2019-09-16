#include "../loam/Visualizer.hpp"

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
    1.5, //epsilon_d
    0.3, //epsilon_n
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

//  PointNormalColor3fVectorCloud noise;
//  noise.resize(7);
//  Vector3f coords_p_1 = Vector3f(-4., 4., 4. );
//  Vector3f coords_p_2 = Vector3f(3., -3., 8. );
//  Vector3f coords_p_3 = Vector3f(-1., 1., 5. );
//  Vector3f coords_p_4 = Vector3f(7., -1., 2. );
//  Vector3f coords_p_5 = Vector3f(-7., -7., 3. );
//  Vector3f coords_p_6 = Vector3f(-1., -7., 4. );
//  Vector3f coords_p_7 = Vector3f(3., -5., 0. );
//  noise[0].coordinates() = coords_p_1;
//  noise[1].coordinates() = coords_p_2;
//  noise[2].coordinates() = coords_p_3;
//  noise[3].coordinates() = coords_p_4;
//  noise[4].coordinates() = coords_p_5;
//  noise[5].coordinates() = coords_p_6;
//  noise[6].coordinates() = coords_p_7;


  PointNormalColor3fVectorCloud p1 = Visualizer::createPlane(
    Vector3f( -35.,-35.,0.),Vector3f( 0.,0.,1.),
    Vector3f( -1.,1.,0.).normalized(), 18, 14, 0.25, 0.25);

  PointNormalColor3fVectorCloud p2 = Visualizer::createPlane(
    Vector3f( 0.,35.,0.),Vector3f( 0.,0.,1.),
    Vector3f( 1.,0.,0.), 18, 14, 0.25, 0.25);

  PointNormalColor3fVectorCloud p3 = Visualizer::createPlane(
    Vector3f( 35.,0.,0.),Vector3f( 0.,0.,1.),
    Vector3f( 0.,1.,0.).normalized(), 18, 14, 0.25, 0.25);

  PointNormalColor3fVectorCloud p4 = Visualizer::createPlane(
    Vector3f( -35.,35.,0.),Vector3f( 0.,0.,1.),
    Vector3f( 1.,1.,0.), 18, 14, 0.25, 0.25);

  PointNormalColor3fVectorCloud p5 = Visualizer::createPlane(
    Vector3f( 35.,-35.,0.),Vector3f( 0.,0.,1.),
    Vector3f( 1.,1.,0.), 18, 14, 0.25, 0.25);


  PointNormalColor3fVectorCloud p6 = Visualizer::createPlane(
    Vector3f( 0.,-35.,0.),Vector3f( 0.,0.,1.),
    Vector3f( 1.,0.,0.), 18, 14, 0.25, 0.25);


  PointNormalColor3fVectorCloud p7 = Visualizer::createPlane(
    Vector3f( -35.,0.,0.),Vector3f( 0.,0.,1.),
    Vector3f( 0.,1.,0.), 18, 14, 0.25, 0.25);

  PointNormalColor3fVectorCloud p8 = Visualizer::createPlane(
    Vector3f( 35.,35.,0.),Vector3f( 0.,0.,1.),
    Vector3f( -1.,1.,0.), 18, 14, 0.25, 0.25);




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

  cloud.insert(
    cloud.end(),
    std::make_move_iterator( p3.begin()),
    std::make_move_iterator( p3.end())
  );
  cloud.insert(
    cloud.end(),
    std::make_move_iterator( p4.begin()),
    std::make_move_iterator( p4.end())
  );

  cloud.insert(
    cloud.end(),
    std::make_move_iterator( p5.begin()),
    std::make_move_iterator( p5.end())
  );

  cloud.insert(
    cloud.end(),
    std::make_move_iterator( p6.begin()),
    std::make_move_iterator( p6.end())
  );

  cloud.insert(
    cloud.end(),
    std::make_move_iterator( p7.begin()),
    std::make_move_iterator( p7.end())
  );

  cloud.insert(
    cloud.end(),
    std::make_move_iterator( p8.begin()),
    std::make_move_iterator( p8.end())
  );


  sph_Image = SphericalDepthImage(cloud,params);
  sph_Image.initializeIndexImage();
  sph_Image.executeOperations();

  std::vector<Matchable> matchables = sph_Image.clusterizeCloud();
    
  RGBImage index_img; 
  RGBImage normals_img;
  RGBImage path_img; 
  RGBImage blurred_normals_img;
  RGBImage clusters_img;


  cv::namedWindow("IndexImage");
  cv::moveWindow("IndexImage", 20, 20);
  cv::namedWindow("NormalsImage");
  cv::moveWindow("NormalsImage", 20, 320);
  cv::namedWindow("PathImage");
  cv::moveWindow("PathImage", 20, 620);
  cv::namedWindow("BlurredNormalsImage");
  cv::moveWindow("BlurredNormalsImage", 20, 920);
  cv::namedWindow("ClustersImage");
  cv::moveWindow("ClustersImage", 20, 1220);


  index_img = sph_Image.drawIndexImg(); 
  normals_img = sph_Image.drawNormalsImg();
  Clusterer clusterer = Clusterer(sph_Image.getPointCloud(), sph_Image.getIndexImage() , params);
  path_img = clusterer.drawPathImg();
  vector<cluster> clusters = clusterer.findClusters();
  blurred_normals_img = clusterer.drawBlurredNormalsImg();
  clusters_img= sph_Image.drawClustersImg( clusters);

  RGBImage index_img_resized; 
  RGBImage normals_img_resized;
  RGBImage path_img_resized; 
  RGBImage blurred_normals_img_resized;
  RGBImage clusters_img_resized;

  const float horizontal_scale= 3;
  const float vertical_scale= 5;
  cv::resize( index_img, index_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);
  cv::resize( normals_img, normals_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);
  cv::resize( path_img, path_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);
  cv::resize( blurred_normals_img, blurred_normals_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);
  cv::resize( clusters_img, clusters_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);

  int counter = 0;
  while( counter < 100){
    ++counter;
    cv::imshow("IndexImage",index_img_resized);
    cv::imshow("NormalsImage",normals_img_resized);
    cv::imshow("PathImage",path_img_resized);
    cv::imshow("BlurredNormalsImage",blurred_normals_img_resized);
    cv::imshow("ClustersImage",clusters_img_resized);
    cv::waitKey(1000);
  }

  return 0;
}
