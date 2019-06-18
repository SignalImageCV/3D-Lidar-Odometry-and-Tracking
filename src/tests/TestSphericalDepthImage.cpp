#include <gtest/gtest.h>
#include "../loam/features/SphericalDepthImage.hpp"

using namespace testing;
namespace Loam{

  class SDIFixture_elevation: public testing::Test {
    protected:
      SphericalDepthImage Sph_Image_1;
      SphericalDepthImage Sph_Image_2;
      SphericalDepthImage Sph_Image_3;
      Point3fVectorCloud cloud_1;
      Point3fVectorCloud cloud_2;
      Point3fVectorCloud cloud_3;
      
      
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
        const int num_rings =2;
        const int num_points_ring= 1;
        const float epsilon_radius= 2;
        const int epsilon_times= 3;
        Sph_Image_1= SphericalDepthImage(
            num_rings,num_points_ring,
            epsilon_radius, epsilon_times,
            cloud_1);
        Sph_Image_2= SphericalDepthImage(
            num_rings,num_points_ring,
            epsilon_radius,epsilon_times,
            cloud_2);
        Sph_Image_3= SphericalDepthImage(
            num_rings,num_points_ring,
            epsilon_radius,epsilon_times,
            cloud_3);
      }
    };

  class SDIFixture_simple2Rings: public testing::Test {
    protected:
      SphericalDepthImage sph_Image;
      Point3fVectorCloud cloud;
      
      
      void SetUp() override {
        cloud.resize(2);
        Vector3f coords_p_1 = Vector3f( 1./2, 0, sqrt(3)/2);
        Vector3f coords_p_2 = Vector3f( sqrt(3)/2, 0, -1./2);
        cloud[0].coordinates() = coords_p_1;
        cloud[1].coordinates() = coords_p_2;
     
        const int num_rings =6;
        const int num_points_ring= 24;
        const float epsilon_radius= 2;
        const int epsilon_times= 3;
     
        sph_Image= SphericalDepthImage(
            num_rings,num_points_ring,
            epsilon_radius, epsilon_times,
            cloud);
      }
   
    };
 

  class SDIFixture_forIndexImage: public testing::Test {
    protected:
      SphericalDepthImage sph_Image;
      Point3fVectorCloud cloud;
      
      
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
     
        const int num_rings =4;
        const int num_points_ring= 24;
        const float epsilon_radius= 2;
        const int epsilon_times= 3;
 
        sph_Image= SphericalDepthImage(
            num_rings,num_points_ring,
            epsilon_radius, epsilon_times,
            cloud);
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

  TEST_F( SDIFixture_forIndexImage, mapBuildIndexImage){
    sph_Image.buildIndexImage();
    vector<vector<list< sphericalDepthPoint >>> index_image_result = sph_Image.getIndexImage();
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
      Point3fVectorCloud cloud;
      
      
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
     
        const int num_rings =10;
        const int num_points_ring= 200;
        const float epsilon_radius= 2;
        const int epsilon_times= 2;
        sph_Image= SphericalDepthImage(
            num_rings,num_points_ring,
            epsilon_radius, epsilon_times,
            cloud);
      }
    };

   TEST_F( SDIFixture_removeFlatSurfaces, removeFlatSurfacesFunc){

    Point3fVectorCloud cloud_raw = sph_Image.getPointCloud();
    ASSERT_EQ( cloud_raw.size(), 7 );
    sph_Image.removeFlatSurfaces();
    Point3fVectorCloud cloud_pruned = sph_Image.getPointCloud();
    ASSERT_EQ( cloud_pruned.size(), 2 );
    }

}

