#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/shell_colors.h>
#include <thread>
#include "loam/features/SphericalDepthImage.hpp"
#include "loam/MyMath.hpp"

//todo remove unused includes



using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;

int main( int argc, char** argv){

  RGBImage index_img; 
  cv::namedWindow("IndexImage");
  cv::moveWindow("IndexImage", 20, 40);
  RGBImage index_img_resized; 
 
  RGBImage boundaries_img; 
  RGBImage boundaries_img_resized; 
  cv::namedWindow("Boundaries");
  cv::moveWindow("Boundaries", 20, 740);

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
  min_elev.coordinates() = Vector3f( 5,5,10);
  cloud.push_back( min_elev);
  PointNormalColor3f max_elev;
  max_elev.coordinates() = Vector3f( 5,5,-10);
  cloud.push_back( max_elev);

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
  sph_Image.removeFlatSurfaces();
  const PointNormalColor3fVectorCloud cloud_after_SurfRemoval=  sph_Image.getPointCloud();
  index_img = sph_Image.drawIndexImg(); 

  sph_Image.collectNormals();
  const PointNormalColor3fVectorCloud cloud_after_NormalComputation=  sph_Image.getPointCloud();

  const float horizontal_scale= 3;
  const float vertical_scale= 3;

  cv::resize( index_img, index_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);
  cv::imshow("IndexImage",index_img_resized);

  for( auto & p : cloud_after_NormalComputation){
    boundaries_img = sph_Image.drawPointNormalBoundaries(p); 
    cv::resize( boundaries_img, boundaries_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);
    cv::imshow("Boundaries",boundaries_img_resized);
    cv::waitKey(1000);
    //std::getchar();

  }

 return 0;
}
