#include <gtest/gtest.h>
#include "../loam/features/IntegralImage.hpp"
#include "../loam/features/SphericalDepthImage.hpp"

using namespace testing;
namespace Loam{

  class IntegralImageFixture : public testing::Test {
    protected:
      PointNormalColor3fVectorCloud cloud;
      vector<vector<DataPoint>> index_image; 

      void SetUp() override {
        cloud.resize(4);
        Vector3f coords_p_1 = Vector3f( 2., 2., -2.);
        Vector3f coords_p_2 = Vector3f( -2., -2., -2.);
        Vector3f coords_p_3 = Vector3f( 3., 3., 3.);
        Vector3f coords_p_4 = Vector3f( 3., 2., 3.);

        cloud[0].coordinates() = coords_p_1;
        cloud[1].coordinates() = coords_p_2;
        cloud[2].coordinates() = coords_p_3;
        cloud[3].coordinates() = coords_p_4;

        const sphericalImage_params params(
          2, //num_vertical_rings
          2, //num_points_ring
          1, //epsilon_times
          1, //epsilon_radius
          1, //depth_differential_threshold
          1,  //min_neighboors_for_normal
          5, //epsilon_c
          0.1, //epsilon_d
          0.02, //epsilon_n
          1, //epsilon_l
          1, //epsilon_dl
          1, //epsilon_p
          1 //epsilon_dp
        );
     
        SphericalDepthImage  sph_Image = SphericalDepthImage(cloud,params);
        sph_Image.initializeIndexImage();
        index_image = sph_Image.getIndexImage();
      }
    };


 TEST( IntegCell, defConstructor){
   IntegralCell cell= IntegralCell();
   ASSERT_TRUE( cell.getPsum() == Eigen::Vector3f::Zero());
   ASSERT_TRUE( cell.getPquad() == Eigen::Matrix3f::Zero());
   ASSERT_TRUE( cell.getPnum() == 0);
 }



 TEST( IntegCell, sumDiffoperator){

    Eigen::Matrix3f m_a;
    m_a <<
      4, 6, 8,
      6, 9, 12,
      8, 12, 16;

    Eigen::Matrix3f m_b;
    m_b <<
      9, 12, 3,
      12, 16, 4,
      3, 4, 1;

    Eigen::Matrix3f m_c;
    m_c <<
      13, 18, 11,
      18, 25, 16,
      11, 16, 17;

    Eigen::Matrix3f m_d;
    m_d <<
      -5, -6, 5,
      -6, -7, 8,
       5,  8, 15;

    IntegralCell cell_a = IntegralCell(
        Eigen::Vector3f( 2.,3.,4.),
        m_a,
        1);
    IntegralCell cell_b = IntegralCell(
        Eigen::Vector3f( 3.,4.,1.),
        m_b,
        1);
    IntegralCell cell_c = IntegralCell(
        Eigen::Vector3f( 5.,7.,5.),
        m_c,
        2);
    IntegralCell cell_d = IntegralCell(
        Eigen::Vector3f( -1.,-1.,3.),
        m_d,
        0);

    ASSERT_TRUE( cell_a + cell_b == cell_c);
    ASSERT_TRUE( cell_a - cell_b == cell_d);
  }



  TEST( IntegCell, equaloperator){
    Eigen::Matrix3f m_a;
    m_a<<
      4, 4, 4,
      4, 4, 4,
      4, 4, 4;
    Eigen::Matrix3f m_b;
    m_b <<
      0, 0, 0,
      0, 0, 0,
      0, 0, 16;
    Eigen::Matrix3f m_c;
    m_c <<
      4, 4, 4,
      4, 4, 4,
      4, 4, 4;



    IntegralCell cell_a = IntegralCell(
        Eigen::Vector3f( -2.,-2.,-2.),
        m_a,
        1);
    IntegralCell cell_z = IntegralCell(
        Eigen::Vector3f( 0.,0.,-4.),
        m_b,
        2);
    IntegralCell cell_b = IntegralCell(
        Eigen::Vector3f( -2.,-2.,-2.),
        m_c,
        1);

    ASSERT_TRUE( cell_a == cell_b);
    ASSERT_FALSE( cell_a == cell_z);
    ASSERT_TRUE( cell_a != cell_z);
    ASSERT_FALSE( cell_a != cell_b);
  }

  TEST_F( IntegralImageFixture, basicContruction){
    IntegralImage integ_img = IntegralImage( cloud , index_image ); 
    integ_img.buildIntegMatrix();
    vector<vector<IntegralCell>> integ_matrix = integ_img.getIntegMat();

    Eigen::Matrix3f m_00;
    m_00 <<
      0, 0, 0,
      0, 0, 0,
      0, 0, 0;
    Eigen::Matrix3f m_01;
    m_01 <<
      18, 15, 18,
      15, 13, 15,
      18, 15, 18;
    Eigen::Matrix3f m_10;
    m_10 <<
      4, 4, 4,
      4, 4, 4,
      4, 4, 4;
    Eigen::Matrix3f m_11;
    m_11 <<
      26, 23, 18,
      23, 21, 15,
      18, 15, 26;

    IntegralCell cell_00 = IntegralCell(
        Eigen::Vector3f( 0.,0.,0.),
        m_00,
        0);
    IntegralCell cell_01 = IntegralCell(
        Eigen::Vector3f( 6.,5.,6.),
        m_01,
        2);
    IntegralCell cell_10 = IntegralCell(
        Eigen::Vector3f( -2.,-2.,-2.),
        m_10,
        1);
    IntegralCell cell_11 = IntegralCell(
        Eigen::Vector3f( 6.,5.,2.),
        m_11,
        4);

    ASSERT_TRUE( integ_matrix[0][0] == cell_00);
    ASSERT_TRUE( integ_matrix[0][1] == cell_01);
    ASSERT_TRUE( integ_matrix[1][0] == cell_10);
    ASSERT_TRUE( integ_matrix[1][1] == cell_11);
  }

  TEST_F( IntegralImageFixture, getCellInsideBounds){

    IntegralImage integ_img = IntegralImage( cloud , index_image ); 
    integ_img.buildIntegMatrix();

    Eigen::Matrix3f m_00;
    m_00 <<
      0, 0, 0,
      0, 0, 0,
      0, 0, 0;
    Eigen::Matrix3f m_00_01;
    m_00_01 <<
      18, 15, 18,
      15, 13, 15,
      18, 15, 18;
    Eigen::Matrix3f m_00_10;
    m_00_10 <<
      4, 4, 4,
      4, 4, 4,
      4, 4, 4;
    Eigen::Matrix3f m_11;
    m_11 <<
      4, 4, -4,
      4, 4, -4,
     -4, -4, 4;

    Eigen::Matrix3f m_10_11 ;
    m_10_11 <<
      8, 8, 0,
      8, 8, 0,
      0, 0, 8;
    Eigen::Matrix3f m_01_11;
    m_01_11 <<
      22, 19, 14,
      19, 17, 11,
      14, 11, 22;

    Eigen::Matrix3f m_00_01_10_11 ;
    m_00_01_10_11 <<
      26, 23, 18,
      23, 21, 15,
      18, 15, 26;



    IntegralCell cell_bounded_00 = IntegralCell(
        Eigen::Vector3f( 0.,0.,0.),
        m_00,
        0);
    IntegralCell cell_bounded_00_01 = IntegralCell(
        Eigen::Vector3f( 6.,5.,6.),
        m_00_01,
        2);
    IntegralCell cell_bounded_00_10 = IntegralCell(
        Eigen::Vector3f( -2.,-2.,-2.),
        m_00_10,
        1);
    IntegralCell cell_bounded_11 = IntegralCell(
        Eigen::Vector3f( 2.,2.,-2.),
        m_11,
        1);

    IntegralCell cell_bounded_10_11 = IntegralCell(
        Eigen::Vector3f( 0.,0.,-4.),
        m_10_11,
        2);
    IntegralCell cell_bounded_01_11 = IntegralCell(
        Eigen::Vector3f( 8.,7.,4.),
        m_01_11,
        3);
    IntegralCell cell_bounded_00_01_10_11 = IntegralCell(
        Eigen::Vector3f( 6.,5.,2.),
        m_00_01_10_11,
        4);



    ASSERT_TRUE( integ_img.getCellInsideBoundaries(0,0,0,0)  == cell_bounded_00);
    ASSERT_TRUE( integ_img.getCellInsideBoundaries(0,0,0,1)  == cell_bounded_00_01);
    ASSERT_TRUE( integ_img.getCellInsideBoundaries(0,0,1,1)  == cell_bounded_00_01);
    ASSERT_TRUE( integ_img.getCellInsideBoundaries(0,1,0,0)  == cell_bounded_00_10);
    ASSERT_TRUE( integ_img.getCellInsideBoundaries(1,1,0,0)  == cell_bounded_00_10);
    ASSERT_TRUE( integ_img.getCellInsideBoundaries(1,1,1,1)  == cell_bounded_11);
    ASSERT_TRUE( integ_img.getCellInsideBoundaries(1,1,0,1)  == cell_bounded_10_11);
    ASSERT_TRUE( integ_img.getCellInsideBoundaries(0,1,1,1)  == cell_bounded_01_11);
    ASSERT_TRUE( integ_img.getCellInsideBoundaries(0,1,0,1)  == cell_bounded_00_01_10_11);
  }
}

