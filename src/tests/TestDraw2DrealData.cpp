#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include "loam/features/SphericalDepthImage.hpp"
#include "loam/DatasetManager.hpp"


using namespace srrg2_core;
using namespace srrg2_core_ros;
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
    
  DatasetManager dM( dataset.value());


  SphericalDepthImage sph_Image;


   
  RGBImage index_img; 
  RGBImage normals_img;
  RGBImage path_img; 
  RGBImage blurred_normals_img;
  RGBImage clusters_img;

  RGBImage index_img_resized; 
  RGBImage normals_img_resized;
  RGBImage path_img_resized; 
  RGBImage blurred_normals_img_resized;
  RGBImage clusters_img_resized;
  const float horizontal_scale= 1;
  const float vertical_scale= 1;
 

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

  PointNormalColor3fVectorCloud cloud = dM.readMessageFromDataset();

  while( cloud.size()> 0){
    sph_Image = SphericalDepthImage(cloud,params);
    sph_Image.initializeIndexImage();
    sph_Image.executeOperations();

    MatchablePtrVecPtr matchablePtrVecPtr = std::make_shared< std::vector< MatchablePtr>>();
    sph_Image.clusterizeCloud( matchablePtrVecPtr);

    index_img = sph_Image.drawIndexImg(); 
    normals_img = sph_Image.drawNormalsImg();
    Clusterer clusterer = Clusterer(sph_Image.getPointCloud(), sph_Image.getIndexImage() , params);
    path_img = clusterer.drawPathImg();
    vector<cluster> clusters = clusterer.findClusters();
    blurred_normals_img = clusterer.drawBlurredNormalsImg();
    clusters_img= sph_Image.drawClustersImg( clusters);

    cv::resize( index_img, index_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);
    cv::resize( normals_img, normals_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);
    cv::resize( path_img, path_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);
    cv::resize( blurred_normals_img, blurred_normals_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);
    cv::resize( clusters_img, clusters_img_resized, cv::Size( 0,0) , horizontal_scale, vertical_scale);

    cv::imshow("IndexImage",index_img_resized);
    cv::imshow("NormalsImage",normals_img_resized);
    cv::imshow("PathImage",path_img_resized);
    cv::imshow("BlurredNormalsImage",blurred_normals_img_resized);
    cv::imshow("ClustersImage",clusters_img_resized);
    cv::waitKey(10);

    cloud = dM.readMessageFromDataset();
  }

  return 0;
}
