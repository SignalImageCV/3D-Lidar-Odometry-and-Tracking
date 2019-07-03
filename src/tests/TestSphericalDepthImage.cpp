#include <gtest/gtest.h>
#include "../loam/features/SphericalDepthImage.hpp"
#include "../loam/Visualizer.hpp"
#include "../loam/features/Line.hpp"
#include "../loam/features/Plane.hpp"


using namespace testing;
namespace Loam{

  class SDIFixture_elevation: public testing::Test {
    protected:
      SphericalDepthImage Sph_Image_1;
      SphericalDepthImage Sph_Image_2;
      SphericalDepthImage Sph_Image_3;
      PointNormalColor3fVectorCloud cloud_1;
      PointNormalColor3fVectorCloud cloud_2;
      PointNormalColor3fVectorCloud cloud_3;
      
      void SetUp() override {
        cloud_1.resize(2);
        cloud_2.resize(2);
        cloud_3.resize(2);
        Vector3f coords_p_1 = Vector3f( 1./2, 0, sqrt(3)/2);
        Vector3f coords_p_2 = Vector3f( sqrt(3)/2, 0, 1./2);
        Vector3f coords_p_3 = Vector3f( sqrt(3)/2, 0, -1./2);
        Vector3f coords_p_4 = Vector3f( 1./2, 0, -sqrt(3)/2);
        cloud_1[0].coordinates() = coords_p_4;
        cloud_1[1].coordinates() = coords_p_3;
        cloud_2[0].coordinates() = coords_p_1;
        cloud_2[1].coordinates() = coords_p_2;
        cloud_3[0].coordinates() = coords_p_1;
        cloud_3[1].coordinates() = coords_p_3;

       
        const sphericalImage_params params(
          2, //num_vertical_rings
          1, //num_points_ring
          3, //epsilon_times
          2, //epsilon_radius
          1, //depth_differential_threshold
          2,  //min_neighboors_for_normal
          5, //epsilon_c
          0.1, //epsilon_d
          0.02, //epsilon_n
          1, //epsilon_l
          1, //epsilon_dl
          1, //epsilon_p
          1 //epsilon_dp
        );
        Sph_Image_1= SphericalDepthImage(cloud_1, params);
        Sph_Image_2= SphericalDepthImage(cloud_2,params);
        Sph_Image_3= SphericalDepthImage(cloud_3,params);
      }
    };

  class SDIFixture_simple2Rings: public testing::Test {
    protected:
      SphericalDepthImage sph_Image;
      PointNormalColor3fVectorCloud cloud;
      
      
      void SetUp() override {
        cloud.resize(2);
        Vector3f coords_p_1 = Vector3f( 1./2, 0, sqrt(3)/2);
        Vector3f coords_p_2 = Vector3f( sqrt(3)/2, 0, -1./2);
        cloud[0].coordinates() = coords_p_1;
        cloud[1].coordinates() = coords_p_2;
     
        const sphericalImage_params params(
          6, //num_vertical_rings
          24, //num_points_ring
          3, //epsilon_times
          2, //epsilon_radius
          1, //depth_differential_threshold
          2, //min_neighboors_for_normal
          5, //epsilon_c
          0.1, //epsilon_d
          0.02, //epsilon_n
          1, //epsilon_l
          1, //epsilon_dl
          1, //epsilon_p
          1 //epsilon_dp
        );
        sph_Image= SphericalDepthImage(cloud,params );
      }
   
    };
 

  class SDIFixture_forIndexImage: public testing::Test {
    protected:
      SphericalDepthImage sph_Image;
      PointNormalColor3fVectorCloud cloud;
      
      
      void SetUp() override{
        cloud.resize(8);
        float epsilon = 0.0001;
        Vector3f coords_p_1 = Vector3f( 1./2, 0, sqrt(3)/2);
        Vector3f coords_p_2 = Vector3f( sqrt(3)/2, 0, -1./2);
        Vector3f coords_p_3 = Vector3f( 0, sqrt(3)/2 , +1./2);
        Vector3f coords_p_4 = Vector3f( 0-epsilon,  sqrt(3)/2 -epsilon, +1./2);
        Vector3f coords_p_5 = Vector3f( 0-epsilon*2, sqrt(3)/2 +epsilon, +1./2);
        Vector3f coords_p_6 = Vector3f( 0-epsilon*3, sqrt(3)/2 -epsilon, +1./2);
        Vector3f coords_p_7 = Vector3f( 0+epsilon, sqrt(3)/2, -1./2);
        Vector3f coords_p_8 = Vector3f( 0-epsilon, sqrt(3)/2, -1./2);
        cloud[0].coordinates() = coords_p_1;
        cloud[1].coordinates() = coords_p_2;
        cloud[2].coordinates() = coords_p_3;
        cloud[3].coordinates() = coords_p_4;
        cloud[4].coordinates() = coords_p_5;
        cloud[5].coordinates() = coords_p_6;
        cloud[6].coordinates() = coords_p_7;
        cloud[7].coordinates() = coords_p_8;
     
        const sphericalImage_params params(
          4, //num_vertical_rings
          24, //num_points_ring
          3, //epsilon_times
          2, //epsilon_radius
          1, //depth_differential_threshold
          2,  //min_neighboors_for_normal
          5, //epsilon_c
          0.1, //epsilon_d
          0.02, //epsilon_n
          1, //epsilon_l
          1, //epsilon_dl
          1, //epsilon_p
          1 //epsilon_dp
        );
        sph_Image= SphericalDepthImage(cloud,params);
      }
    };

 
  TEST_F( SDIFixture_elevation, extimateMinMaxElev){


    Vector2f result_1 = SphericalDepthImage::extimateMinMaxElevation( cloud_1);
    Vector2f truth_1 = Vector2f( 2*M_PI/3, 5*M_PI/6);
    ASSERT_EQ( result_1.x(), truth_1.x());
    ASSERT_EQ( result_1.y(), truth_1.y());

    Vector2f result_2 = SphericalDepthImage::extimateMinMaxElevation( cloud_2);
    Vector2f truth_2 = Vector2f( M_PI/6, M_PI/3);
    ASSERT_EQ( result_2.x(), truth_2.x());
    ASSERT_EQ( result_2.y(), truth_2.y());

    Vector2f result_3 = SphericalDepthImage::extimateMinMaxElevation( cloud_3);
    Vector2f truth_3 = Vector2f( M_PI/6, 2*M_PI/3);
    ASSERT_EQ( result_3.x(), truth_3.x());
    ASSERT_EQ( result_3.y(), truth_3.y());

  }
 
  TEST_F( SDIFixture_simple2Rings, mapSphericalCoords){

    const float degree = M_PI/180;
    const float azimuth_1 = degree*10;
    const float elevation_1 = degree*31;
    vector<int> result_1 =  sph_Image.mapSphericalCoordsInIndexImage( azimuth_1, elevation_1);
    ASSERT_EQ( result_1[0], 0);
    ASSERT_EQ( result_1[1], 12);

    const float azimuth_2 = degree*50;
    const float elevation_2 = degree*80;
    vector<int> result_2 =  sph_Image.mapSphericalCoordsInIndexImage( azimuth_2, elevation_2);
    ASSERT_EQ( result_2[0], 3);
    ASSERT_EQ( result_2[1], 15);

    const float azimuth_3 = - degree*70;
    const float elevation_3 = degree*89;
    vector<int> result_3 =  sph_Image.mapSphericalCoordsInIndexImage( azimuth_3, elevation_3);
    ASSERT_EQ( result_3[0], 3);
    ASSERT_EQ( result_3[1], 7);

    const float azimuth_4 = - degree*170;
    const float elevation_4 = degree*41;
    vector<int> result_4 =  sph_Image.mapSphericalCoordsInIndexImage( azimuth_4, elevation_4);
    ASSERT_EQ( result_4[0], 0);
    ASSERT_EQ( result_4[1], 0);

    const float azimuth_5 = - degree*170;
    const float elevation_5 = degree*(30+89);
    vector<int> result_5 =  sph_Image.mapSphericalCoordsInIndexImage( azimuth_5, elevation_5);
    ASSERT_EQ( result_5[0], 5);
    ASSERT_EQ( result_5[1], 0);

  }

  TEST_F( SDIFixture_forIndexImage, mapInitializeIndexImage){
    sph_Image.initializeIndexImage();
    vector<vector<list< DataPoint >>> index_image_result = sph_Image.getIndexImage();
    ASSERT_TRUE( index_image_result.size() > 0 );
    ASSERT_EQ( index_image_result.size(), 4 );
    ASSERT_TRUE( index_image_result[0].size() >0 );
    ASSERT_EQ( index_image_result[0].size() , 24 );


    //p1
    ASSERT_EQ( index_image_result[0][12].size(),1 );
    ASSERT_EQ( index_image_result[0][11].size(),0 );
    ASSERT_EQ( index_image_result[0][13].size(),0 );
    ASSERT_EQ( index_image_result[1][12].size(),0 );
    //p2
    ASSERT_EQ( index_image_result[3][12].size(),1 );
    ASSERT_EQ( index_image_result[2][12].size(),0 );
    ASSERT_EQ( index_image_result[3][11].size(),0 );
    ASSERT_EQ( index_image_result[3][13].size(),0 );
    //p3,p4,p5,p6
    ASSERT_EQ( index_image_result[1][18].size(),4 );
    ASSERT_EQ( index_image_result[0][18].size(),0 );
    ASSERT_EQ( index_image_result[2][18].size(),0 );
    ASSERT_EQ( index_image_result[1][19].size(),0 );
    ASSERT_EQ( index_image_result[1][17].size(),0 );
    //p7,p8
    ASSERT_EQ( index_image_result[3][17].size(),1 );
    ASSERT_EQ( index_image_result[2][17].size(),0 );
    ASSERT_EQ( index_image_result[3][16].size(),0 );
    ASSERT_EQ( index_image_result[3][18].size(),1 );
    ASSERT_EQ( index_image_result[2][18].size(),0 );
    ASSERT_EQ( index_image_result[3][19].size(),0 );

  }

  TEST( SphericalDepthImage, directMapping){
    Vector3f cartesian_1 = Vector3f( 1./2, 0, -sqrt(3)/2);
    Vector3f spherical_truth_1 = Vector3f( 0, 5*M_PI/6, 1);
    Vector3f result_1 = SphericalDepthImage::directMappingFunc(cartesian_1 );
    ASSERT_NEAR( spherical_truth_1.x(), result_1.x(), 1e-3);
    ASSERT_NEAR( spherical_truth_1.y(), result_1.y(), 1e-3);
    ASSERT_NEAR( spherical_truth_1.z(), result_1.z(), 1e-3);

    Vector3f cartesian_2 = Vector3f(0,-sqrt(3)/2, 1./2);
    Vector3f spherical_truth_2 = Vector3f(-M_PI/2, M_PI/3, 1);
    Vector3f result_2 = SphericalDepthImage::directMappingFunc(cartesian_2 );
    ASSERT_NEAR( spherical_truth_2.x(), result_2.x(), 1e-3);
    ASSERT_NEAR( spherical_truth_2.y(), result_2.y(), 1e-3);
    ASSERT_NEAR( spherical_truth_2.z(), result_2.z(), 1e-3);


    Vector3f cartesian_3 = Vector3f(1, 0, 0);
    Vector3f spherical_truth_3 = Vector3f(0, M_PI/2, 1);
    Vector3f result_3 = SphericalDepthImage::directMappingFunc(cartesian_3 );
    ASSERT_NEAR( spherical_truth_3.x(), result_3.x(), 1e-3);
    ASSERT_NEAR( spherical_truth_3.y(), result_3.y(), 1e-3);
    ASSERT_NEAR( spherical_truth_3.z(), result_3.z(), 1e-3);

   
    Vector3f cartesian_4 = Vector3f(sqrt(3)/2, 0, -1./2);
    Vector3f spherical_truth_4 = Vector3f(0, 2*M_PI/3,1);
    Vector3f result_4 = SphericalDepthImage::directMappingFunc(cartesian_4 );
    ASSERT_NEAR( spherical_truth_4.x(), result_4.x(), 1e-3);
    ASSERT_NEAR( spherical_truth_4.y(), result_4.y(), 1e-3);
    ASSERT_NEAR( spherical_truth_4.z(), result_4.z(), 1e-3);

    Vector3f cartesian_5 = Vector3f( 0 ,sqrt(3)/2, -1./2);
    Vector3f spherical_truth_5 = Vector3f(M_PI/2, 2*M_PI/3,1);
    Vector3f result_5 = SphericalDepthImage::directMappingFunc(cartesian_5 );
    ASSERT_NEAR( spherical_truth_5.x(), result_5.x(), 1e-3);
    ASSERT_NEAR( spherical_truth_5.y(), result_5.y(), 1e-3);
    ASSERT_NEAR( spherical_truth_5.z(), result_5.z(), 1e-3);

  }

  TEST( SphericalDepthImage, inverseMapping){
    Vector3f spherical_1= Vector3f( 0, 5*M_PI/6, 1);
    Vector3f cartesian_truth_1 = Vector3f(1./2, 0, -sqrt(3)/2);
    Vector3f result_1 = SphericalDepthImage::inverseMappingFunc(spherical_1);
    ASSERT_NEAR(cartesian_truth_1.x(), result_1.x(), 1e-3);
    ASSERT_NEAR(cartesian_truth_1.y(), result_1.y(), 1e-3);
    ASSERT_NEAR(cartesian_truth_1.z(), result_1.z(), 1e-3);

    Vector3f spherical_2= Vector3f( -M_PI/2, M_PI/3, 1);
    Vector3f cartesian_truth_2 = Vector3f(0,-sqrt(3)/2, 1./2);
    Vector3f result_2 = SphericalDepthImage::inverseMappingFunc(spherical_2);
    ASSERT_NEAR(cartesian_truth_2.x(), result_2.x(), 1e-3);
    ASSERT_NEAR(cartesian_truth_2.y(), result_2.y(), 1e-3);
    ASSERT_NEAR(cartesian_truth_2.z(), result_2.z(), 1e-3);
 
    Vector3f spherical_3= Vector3f( 0, M_PI/2, 1);
    Vector3f cartesian_truth_3 = Vector3f(1,0,0);
    Vector3f result_3 = SphericalDepthImage::inverseMappingFunc(spherical_3);
    ASSERT_NEAR(cartesian_truth_3.x(), result_3.x(), 1e-3);
    ASSERT_NEAR(cartesian_truth_3.y(), result_3.y(), 1e-3);
    ASSERT_NEAR(cartesian_truth_3.z(), result_3.z(), 1e-3);

    Vector3f spherical_4= Vector3f( 0, 2*M_PI/3, 1);
    Vector3f cartesian_truth_4 = Vector3f(sqrt(3)/2,0 ,-1./2);
    Vector3f result_4 = SphericalDepthImage::inverseMappingFunc(spherical_4);
    ASSERT_NEAR(cartesian_truth_4.x(), result_4.x(), 1e-3);
    ASSERT_NEAR(cartesian_truth_4.y(), result_4.y(), 1e-3);
    ASSERT_NEAR(cartesian_truth_4.z(), result_4.z(), 1e-3);
 

    Vector3f spherical_5= Vector3f(M_PI/2, 2*M_PI/3, 1);
    Vector3f cartesian_truth_5 = Vector3f(0 ,sqrt(3)/2, -1./2);
    Vector3f result_5 = SphericalDepthImage::inverseMappingFunc(spherical_5);
    ASSERT_NEAR(cartesian_truth_5.x(), result_5.x(), 1e-3);
    ASSERT_NEAR(cartesian_truth_5.y(), result_5.y(), 1e-3);
    ASSERT_NEAR(cartesian_truth_5.z(), result_5.z(), 1e-3);

  }

  class SDIFixture_removeFlatSurfaces: public testing::Test {
    protected:
      SphericalDepthImage sph_Image;
      PointNormalColor3fVectorCloud cloud;
      
      
      void SetUp() override{
        cloud.resize(7);
        Vector3f coords_p_1 = Vector3f(4., 4., 4. );
        Vector3f coords_p_2 = Vector3f(3., 3., 8. );
        Vector3f coords_p_3 = Vector3f(1., 1., 5. );
        Vector3f coords_p_4 = Vector3f(7., 1., 2. );
        Vector3f coords_p_5 = Vector3f(7., 7., 3. );
        Vector3f coords_p_6 = Vector3f(1., 7., 4. );
        Vector3f coords_p_7 = Vector3f(3., 5., 0. );
        cloud[0].coordinates() = coords_p_1;
        cloud[1].coordinates() = coords_p_2;
        cloud[2].coordinates() = coords_p_3;
        cloud[3].coordinates() = coords_p_4;
        cloud[4].coordinates() = coords_p_5;
        cloud[5].coordinates() = coords_p_6;
        cloud[6].coordinates() = coords_p_7;
     
        const sphericalImage_params params(
          10, //num_vertical_rings
          200, //num_points_ring
          2, //epsilon_times
          2, //epsilon_radius
          1, //depth_differential_threshold
          2,  //min_neighboors_for_normal
          5, //epsilon_c
          0.1, //epsilon_d
          0.02, //epsilon_n
          1, //epsilon_l
          1, //epsilon_dl
          1, //epsilon_p
          1 //epsilon_dp
        );
      
        sph_Image= SphericalDepthImage(cloud,params);
      }
    };

  TEST_F( SDIFixture_removeFlatSurfaces, removeFlatSurfacesFunc){

    PointNormalColor3fVectorCloud cloud_raw = sph_Image.getPointCloud();
    ASSERT_EQ( cloud_raw.size(), 7 );
    sph_Image.removeFlatSurfaces();
    PointNormalColor3fVectorCloud cloud_pruned = sph_Image.getPointCloud();
    ASSERT_EQ( cloud_pruned.size(), 2 );
   }
  
  class SDIFixture_normalComputation: public testing::Test {
    protected:
      SphericalDepthImage sph_Image_horizontal;
      PointNormalColor3fVectorCloud cloud_horizontal;

      SphericalDepthImage sph_Image_vertical;
      PointNormalColor3fVectorCloud cloud_vertical;
          
      
      void SetUp() override{
        cloud_horizontal.resize(7);
        Vector3f coords_p_1_h = Vector3f(1., 1., 0. );
        Vector3f coords_p_2_h = Vector3f(1., tan( M_PI/3), 0. );
        Vector3f coords_p_3_h = Vector3f(1., 0., 0. );
        Vector3f coords_p_4_h = Vector3f(1., -1., 0. );
        Vector3f coords_p_5_h = Vector3f(2., 2*tan(M_PI/4 + M_PI/12) , 0. );
        Vector3f coords_p_6_h = Vector3f(2., 0., 2. );
        Vector3f coords_p_7_h = Vector3f(2., 0., -2. );

        cloud_horizontal[0].coordinates() = coords_p_1_h;
        cloud_horizontal[1].coordinates() = coords_p_2_h;
        cloud_horizontal[2].coordinates() = coords_p_3_h;
        cloud_horizontal[3].coordinates() = coords_p_4_h;
        cloud_horizontal[4].coordinates() = coords_p_5_h;
        cloud_horizontal[5].coordinates() = coords_p_6_h;
        cloud_horizontal[6].coordinates() = coords_p_7_h;

        cloud_vertical.resize(5);
        Vector3f coords_p_1_v = Vector3f(1., 0., 1. );
        Vector3f coords_p_2_v = Vector3f(1., 0., tan( M_PI/3) );
        Vector3f coords_p_3_v = Vector3f(1., 0., 0. );
        Vector3f coords_p_4_v = Vector3f(1., 0., -1. );
        Vector3f coords_p_5_v = Vector3f(2., 0., 2*tan(M_PI/4 + M_PI/24) );
    
        cloud_vertical[0].coordinates() = coords_p_1_v;
        cloud_vertical[1].coordinates() = coords_p_2_v;
        cloud_vertical[2].coordinates() = coords_p_3_v;
        cloud_vertical[3].coordinates() = coords_p_4_v;
        cloud_vertical[4].coordinates() = coords_p_5_v;
        const sphericalImage_params params(
          105, //num_vertical_rings
          360, //num_points_ring
          0, //epsilon_times
          0, //epsilon_radius
          1, //depth_differential_threshold
          2,  //min_neighboors_for_normal
          5, //epsilon_c
          0.1, //epsilon_d
          0.02, //epsilon_n
          1, //epsilon_l
          1, //epsilon_dl
          1, //epsilon_p
          1 //epsilon_dp
        );
      


        sph_Image_horizontal= SphericalDepthImage(cloud_horizontal,params);

        sph_Image_vertical= SphericalDepthImage(cloud_vertical,params);
      }
    };

  TEST_F( SDIFixture_normalComputation, findBoundariesHorizontalLine){
    sph_Image_horizontal.initializeIndexImage();
    sph_Image_horizontal.discoverNormalsBoundaryIndexes();

    vector<int> indexes_starting_point=  sph_Image_horizontal.mapCartesianCoordsInIndexImage(cloud_horizontal[0].coordinates());

    vector<int> indexes_first_point_in_boundaries =  sph_Image_horizontal.mapCartesianCoordsInIndexImage(cloud_horizontal[3].coordinates());
    int index_first_column_in_boundaries = indexes_first_point_in_boundaries[1];
    vector<int> indexes_first_point_out_boundaries =  sph_Image_horizontal.mapCartesianCoordsInIndexImage(cloud_horizontal[4].coordinates());
    int index_first_column_out_boundaries = indexes_first_point_out_boundaries[1];
  
    list<DataPoint>  list = sph_Image_horizontal.getListDataPointsAt(indexes_starting_point[0],indexes_starting_point[1]);


    ASSERT_EQ( list.size(), 1 );
    if( list.size() ==1){
      DataPoint starting_point = list.front();
      vector<int> bounds = starting_point.getBoundaries();

      ASSERT_EQ( bounds[2] , index_first_column_in_boundaries );
      ASSERT_EQ( bounds[3] , index_first_column_out_boundaries -1);
    }
 }

  TEST_F( SDIFixture_normalComputation, findBoundariesVerticalLine){
    sph_Image_vertical.initializeIndexImage();
    sph_Image_vertical.discoverNormalsBoundaryIndexes();

    vector<int> indexes_starting_point=  sph_Image_vertical.mapCartesianCoordsInIndexImage(cloud_vertical[0].coordinates());

    vector<int> indexes_last_point_out_boundaries =  sph_Image_vertical.mapCartesianCoordsInIndexImage(cloud_vertical[4].coordinates());
    int index_last_row_out_boundaries = indexes_last_point_out_boundaries[0];

    vector<int> indexes_last_point_in_boundaries =  sph_Image_vertical.mapCartesianCoordsInIndexImage(cloud_vertical[3].coordinates());
    int index_last_row_in_boundaries = indexes_last_point_in_boundaries[0];

    list<DataPoint>  list = sph_Image_vertical.getListDataPointsAt(indexes_starting_point[0],indexes_starting_point[1]);

    ASSERT_EQ( list.size(), 1 );
    if( list.size() ==1){
      DataPoint starting_point = list.front();
      vector<int> bounds = starting_point.getBoundaries();

      ASSERT_EQ( bounds[0] , index_last_row_out_boundaries+1);
      ASSERT_EQ( bounds[1] , index_last_row_in_boundaries);
    }
    sph_Image_vertical.removePointsWithoutNormal();
    ASSERT_EQ( sph_Image_vertical.getPointCloud().size(), 3);
  }

  class SDIFixture_clustering: public testing::Test {
    protected:
      SphericalDepthImage sph_Image;
      PointNormalColor3fVectorCloud cloud;

      void SetUp() override{

        const sphericalImage_params params(
          105, //num_vertical_rings
          360, //num_points_ring
          0, //epsilon_times
          0, //epsilon_radius
          1, //depth_differential_threshold
          4,  //min_neighboors_for_normal
          5, //epsilon_c
          0.1, //epsilon_d
          0.02, //epsilon_n
          1, //epsilon_l
          1, //epsilon_dl
          1, //epsilon_p
          1 //epsilon_dp
        );


        PointNormalColor3fVectorCloud noise;
        noise.resize(7);
        Vector3f coords_p_1 = Vector3f(4., 4., 4. );
        Vector3f coords_p_2 = Vector3f(3., 3., 8. );
        Vector3f coords_p_3 = Vector3f(1., 1., 5. );
        Vector3f coords_p_4 = Vector3f(7., 1., 2. );
        Vector3f coords_p_5 = Vector3f(7., 7., 3. );
        Vector3f coords_p_6 = Vector3f(1., 7., 4. );
        Vector3f coords_p_7 = Vector3f(3., 5., 0. );
        noise[0].coordinates() = coords_p_1;
        noise[1].coordinates() = coords_p_2;
        noise[2].coordinates() = coords_p_3;
        noise[3].coordinates() = coords_p_4;
        noise[4].coordinates() = coords_p_5;
        noise[5].coordinates() = coords_p_6;
        noise[6].coordinates() = coords_p_7;
 

        PointNormalColor3fVectorCloud l1 = Visualizer::createLine(
            Vector3f( 2.,3.,1.), Vector3f( 0.,0.,1.), 2, 0.5);
        PointNormalColor3fVectorCloud l2 = Visualizer::createLine(
            Vector3f( -4.,-2.,-2.), Vector3f( 0.,0.,-1.), 6, 1);
        PointNormalColor3fVectorCloud p1 = Visualizer::createPlane(
            Vector3f( 2.,-7.,0.), Vector3f( 1.,0.,0.),
            Vector3f( 0.,0.,1.), 2, 3, 0.5, 0.5);
  

        //cloud.insert(
         // cloud.end(),
         // std::make_move_iterator( l1.begin()),
         // std::make_move_iterator( l1.end())
         // );

        //cloud.insert(
         // cloud.end(),
         // std::make_move_iterator( l2.begin()),
         // std::make_move_iterator( l2.end())
         // );

        cloud.insert(
          cloud.end(),
          std::make_move_iterator( p1.begin()),
          std::make_move_iterator( p1.end())
          );

        sph_Image = SphericalDepthImage(cloud,params);

      }

    };

  TEST_F( SDIFixture_clustering, clusterizeFeatures){
    sph_Image.initializeIndexImage();
    IntegralImage integ_img =  sph_Image.collectNormals();
    std::vector<Matchable> matchables = sph_Image.clusterizeCloud( integ_img);

    ASSERT_EQ( matchables.size(), 2 );
  }
}

