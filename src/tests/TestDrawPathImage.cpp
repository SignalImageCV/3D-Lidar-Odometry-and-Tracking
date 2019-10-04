#include <srrg_system_utils/parse_command_line.h>
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
  ArgumentString  num_vertical_rings(&cmd_line, "vr", "num_vertical_rings", "num of vertical rings" , "");
  ArgumentString  num_points_ring(&cmd_line, "hr", "num_points_ring", "num of horizontal rings (slices)" , "");
  ArgumentString  epsilon_t(&cmd_line, "et", "epsilon_t", "min num of times points have to fall in the base circle to be considered vertical " , "");
  ArgumentString  epsilon_r(&cmd_line, "er", "epsilon_r", "min radius of the circle in which vertical points have to fall" , "");
  ArgumentString  depth_differential_threshold(&cmd_line, "dd", "depth_differential_threshold", "max difference between depths in near points to be considered valid points" , "");
  ArgumentString  min_neighboors_for_normal(&cmd_line, "nn", "min_neighboors_for_normal", "min num of points to use to compute the normals" , "");
  ArgumentString  epsilon_c(&cmd_line, "ec", "epsilon_c", "min num of points that forma a cluster" , "");
  ArgumentString  epsilon_d(&cmd_line, "ed", "epsilon_d", "min cartesian distance between points of the same cluster" , ""); //currently unused
  ArgumentString  epsilon_n(&cmd_line, "en", "epsilon_n", "min distance between directions of normals of points in the same cluster" , "");
  ArgumentString  epsilon_l(&cmd_line, "el", "epsilon_l", "min number that descibes the eigenvalue constraint of a line" , "");
  ArgumentString  epsilon_dl(&cmd_line,"edl", "epsilon_dl", "min number of the cumulative cartesian error of the matchable fitted to a line" , "");
  ArgumentString  epsilon_p(&cmd_line, "ep", "epsilon_p", "min number that descibes the eigenvalue constraint of a plane" , "");
  ArgumentString  epsilon_dp(&cmd_line,"edp", "epsilon_dp", "min number of the cumulative cartesian error of the matchable fitted to a plane" , "");
  cmd_line.parse();

  messages_registerTypes();
  srrgInit( argc, argv, "hi");

  const sphericalImage_params params(
    std::stoi( num_vertical_rings.value()), 
    std::stoi( num_points_ring.value()),
    std::stoi( epsilon_t.value()),
    std::stof( epsilon_r.value()),
    std::stof( depth_differential_threshold.value()),
    std::stoi( min_neighboors_for_normal.value()),
    std::stoi( epsilon_c.value()),
    std::stof( epsilon_d.value()),
    std::stof( epsilon_n.value()),
    std::stof( epsilon_l.value()),
    std::stof( epsilon_dl.value()),
    std::stof( epsilon_p.value()),
    std::stof( epsilon_dp.value())
  );

  SphericalDepthImage sph_Image;
  DatasetManager dM( dataset.value());
  RGBImage index_img; 
  RGBImage normals_img;
  RGBImage path_img;
  RGBImage blurred_normals_img;
  RGBImage clusters_img;
 

  cv::namedWindow("IndexImage");
  cv::moveWindow("IndexImage", 10, 40);

  cv::namedWindow("NormalsImage");
  cv::moveWindow("NormalsImage", 10, 240);

  cv::namedWindow("PathImage");
  cv::moveWindow("PathImage", 10, 440);

  cv::namedWindow("BlurredNormalsImage");
  cv::moveWindow("BlurredNormalsImage", 10, 640);

  cv::namedWindow("ClustersImage");
  cv::moveWindow("ClustersImage", 10, 840);




  PointNormalColor3fVectorCloud current_point_cloud= dM.readMessageFromDataset();

  while( current_point_cloud.size()> 0){
 
    sph_Image= SphericalDepthImage(current_point_cloud,params);
    sph_Image.initializeIndexImage();
    sph_Image.removeFlatSurfaces();
    sph_Image.collectNormals();
    MatchablePtrVecPtr matchablePtrVecPtr = std::make_shared< std::vector< MatchablePtr>>();
    sph_Image.clusterizeCloud( matchablePtrVecPtr);

    index_img = sph_Image.drawIndexImg(); 
    normals_img = sph_Image.drawNormalsImg();
    clusters_img= sph_Image.drawClustersImg();

    Clusterer c= Clusterer( sph_Image.getPointCloud(), sph_Image.getIndexImage(), params);
    path_img = c.drawPathImg();
    c.blurNormals();
    blurred_normals_img = c.drawBlurredNormalsImg();



    cv::imshow("IndexImage",index_img);
    cv::imshow("NormalsImage",normals_img);
    cv::imshow("PathImage",path_img);
    cv::imshow("BlurredNormalsImage",blurred_normals_img);
    cv::imshow("ClustersImage",clusters_img);
   
    cv::waitKey();

    current_point_cloud = dM.readMessageFromDataset();
  }
  return 0;
}
