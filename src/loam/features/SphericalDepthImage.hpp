#pragma once
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include <random>

#include "opencv2/opencv.hpp"

#include "DataPoint.hpp"
#include "IntegralImage.hpp"
#include "Line.hpp"
#include "Plane.hpp"

using namespace std;
using namespace srrg2_core;


namespace Loam{

  typedef struct sphericalImage_params_tag{
     int num_vertical_rings;
     int num_points_ring;
     int epsilon_times;
     float epsilon_radius;
     float depth_differential_threshold;
     int min_neighboors_for_normal;
     int epsilon_c;
     float epsilon_d;
     float epsilon_n;
     float epsilon_l;
     float epsilon_dl;
     float epsilon_p;
     float epsilon_dp;
     sphericalImage_params_tag(
         const int t_num_vertical_rings,
         const int t_num_points_ring,
         const int t_epsilon_times,
         const float t_epsilon_radius,
         const float t_depth_differential_threshold,
         const int t_min_neighboors_for_normal,
         const int t_epsilon_c,
         const float t_epsilon_d,
         const float t_epsilon_n,
         const float t_epsilon_l,
         const float t_epsilon_dl,
         const float t_epsilon_p,
         const float t_epsilon_dp
         ):
       num_vertical_rings( t_num_vertical_rings),
       num_points_ring( t_num_points_ring),
       epsilon_times( t_epsilon_times),
       epsilon_radius( t_epsilon_radius),
       depth_differential_threshold(
           t_depth_differential_threshold),
       min_neighboors_for_normal(
           t_min_neighboors_for_normal),
       epsilon_c( t_epsilon_c),
       epsilon_d( t_epsilon_d),
       epsilon_n( t_epsilon_n),
       epsilon_l( t_epsilon_l),
       epsilon_dl( t_epsilon_dl),
       epsilon_p( t_epsilon_p),
       epsilon_dp( t_epsilon_dp)
    {}
     sphericalImage_params_tag():
       num_vertical_rings( -1),
       num_points_ring( -1),
       epsilon_times( -1),
       epsilon_radius( -1),
       depth_differential_threshold(-1),
       min_neighboors_for_normal(-1),
       epsilon_c(-1),
       epsilon_d(-1),
       epsilon_n(-1),
       epsilon_l(-1),
       epsilon_dl(-1),
       epsilon_p(-1),
       epsilon_dp(-1)
    {}
  }sphericalImage_params;


  typedef cv::Mat_< cv::Vec3b > RGBImage;

  class SphericalDepthImage {
     protected:
       //this matrix is both the index matrix I and the spherical depth matrix D
       vector<vector< list< DataPoint >>> m_index_image;
       PointNormalColor3fVectorCloud m_cloud;
       sphericalImage_params m_params;
       float m_min_elevation;
       float m_max_elevation;
       RGBImage m_drawing_index_img;
       RGBImage m_drawing_normals;
       RGBImage m_drawing_clusters;
      

     public:
      SphericalDepthImage()= default;

      SphericalDepthImage(
          const PointNormalColor3fVectorCloud & cloud,
          const sphericalImage_params t_params);

      ~SphericalDepthImage() = default;

      void executeOperations();

      void removeFlatSurfaces();
      IntegralImage collectNormals();


      void initializeIndexImage();
      void resetIndexImage();

      void markVerticalPoints();

      void removeNonVerticalPoints();
      void removePointsWithoutNormal();


      bool expandNormalBoundariesUp(DataPoint & t_starting_point,int & t_neighboors_count);
      bool expandNormalBoundariesDown(DataPoint & t_starting_point,int & t_neighboors_count);
      bool expandNormalBoundariesLeft(DataPoint & t_starting_point,int & t_neighboors_count);
      bool expandNormalBoundariesRight(DataPoint & t_starting_point,int & t_neighboors_count);
      void discoverNormalsBoundaryIndexes();

      IntegralImage computePointNormals();


  






      ///////////////////////////////////

      vector<Matchable> clusterizeCloud(IntegralImage & t_integ_img);

      vector< vector< int>> findGoodClusterSeeds();

      bool expandClusterBoundariesUp( DataPoint & t_seed_point,int & t_included_points_count);
      bool expandClusterBoundariesDown( DataPoint & t_seed_point,int & t_included_points_count);
      bool expandClusterBoundariesLeft( DataPoint & t_seed_point,int & t_included_points_count);
      bool expandClusterBoundariesRight( DataPoint & t_seed_point,int & t_included_points_count);
      bool discoverClusterBoundaryIndexes(
          const int t_seed_row, const int t_seed_col, const int t_seed_list_position);

      //////////////////////////////

      void markClusteredPoints(DataPoint & t_seed_point);

      vector<int> fetchGoodSeedIndexes();

      int countPointsIndexImage();
      int countPointsNotClustered();

      PointNormalColor3fVectorCloud fetchPointsInBoundaries(
          const int t_rowMin,const int t_rowMax,const int t_colMin,const int t_colMax);
 


      vector<int> mapSphericalCoordsInIndexImage(
          const float t_azimuth, const float t_elevation);

      vector<int> mapCartesianCoordsInIndexImage(
          const Vector3f & t_coords);
 
      static Vector2f extimateMinMaxElevation( const PointNormalColor3fVectorCloud & cloud);

      static  Vector3f directMappingFunc(const Vector3f & t_cart_coords);

      static  Vector3f inverseMappingFunc(const Vector3f & t_spher_coords);


      RGBImage drawIndexImg();
      RGBImage drawNormalsImg();
      RGBImage drawClustersImg();



      inline const vector<vector<list<DataPoint>>> & getIndexImage(){ return m_index_image;};

      inline void setIndexImage( const vector<vector<list<DataPoint>>> & t_indexImage){ m_index_image = t_indexImage;};

      inline const PointNormalColor3fVectorCloud & getPointCloud(){ return m_cloud;};

      inline void setPointCloud( const PointNormalColor3fVectorCloud & t_cloud){ m_cloud = t_cloud;};

      inline list<DataPoint> getListDataPointsAt( const int t_row, const int t_col){ return m_index_image[t_row][t_col];};
   };
}


