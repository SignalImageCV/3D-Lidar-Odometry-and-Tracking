#pragma once
#include <srrg_system_utils/system_utils.h>
#include <gtest/gtest.h>
#include <stdio.h>
#include "../loam/MyMath.hpp"


namespace Loam{
  using namespace testing;
  using namespace srrg2_core;
  using namespace Eigen;

  TEST( SphericalDepthImage, directMapping){
    Vector3f cartesian_1 = Vector3f( 1./2, 0, -sqrt(3)/2);
    Vector3f spherical_truth_1 = Vector3f( 0, 5*M_PI/6, 1);
    Vector3f result_1 = MyMath::directMappingFunc(cartesian_1 );
    ASSERT_NEAR( spherical_truth_1.x(), result_1.x(), 1e-3);
    ASSERT_NEAR( spherical_truth_1.y(), result_1.y(), 1e-3);
    ASSERT_NEAR( spherical_truth_1.z(), result_1.z(), 1e-3);

    Vector3f cartesian_2 = Vector3f(0,-sqrt(3)/2, 1./2);
    Vector3f spherical_truth_2 = Vector3f(-M_PI/2, M_PI/3, 1);
    Vector3f result_2 = MyMath::directMappingFunc(cartesian_2 );
    ASSERT_NEAR( spherical_truth_2.x(), result_2.x(), 1e-3);
    ASSERT_NEAR( spherical_truth_2.y(), result_2.y(), 1e-3);
    ASSERT_NEAR( spherical_truth_2.z(), result_2.z(), 1e-3);


    Vector3f cartesian_3 = Vector3f(1, 0, 0);
    Vector3f spherical_truth_3 = Vector3f(0, M_PI/2, 1);
    Vector3f result_3 = MyMath::directMappingFunc(cartesian_3 );
    ASSERT_NEAR( spherical_truth_3.x(), result_3.x(), 1e-3);
    ASSERT_NEAR( spherical_truth_3.y(), result_3.y(), 1e-3);
    ASSERT_NEAR( spherical_truth_3.z(), result_3.z(), 1e-3);

   
    Vector3f cartesian_4 = Vector3f(sqrt(3)/2, 0, -1./2);
    Vector3f spherical_truth_4 = Vector3f(0, 2*M_PI/3,1);
    Vector3f result_4 = MyMath::directMappingFunc(cartesian_4 );
    ASSERT_NEAR( spherical_truth_4.x(), result_4.x(), 1e-3);
    ASSERT_NEAR( spherical_truth_4.y(), result_4.y(), 1e-3);
    ASSERT_NEAR( spherical_truth_4.z(), result_4.z(), 1e-3);

    Vector3f cartesian_5 = Vector3f( 0 ,sqrt(3)/2, -1./2);
    Vector3f spherical_truth_5 = Vector3f(M_PI/2, 2*M_PI/3,1);
    Vector3f result_5 = MyMath::directMappingFunc(cartesian_5 );
    ASSERT_NEAR( spherical_truth_5.x(), result_5.x(), 1e-3);
    ASSERT_NEAR( spherical_truth_5.y(), result_5.y(), 1e-3);
    ASSERT_NEAR( spherical_truth_5.z(), result_5.z(), 1e-3);

  }

  TEST( SphericalDepthImage, inverseMapping){
    Vector3f spherical_1= Vector3f( 0, 5*M_PI/6, 1);
    Vector3f cartesian_truth_1 = Vector3f(1./2, 0, -sqrt(3)/2);
    Vector3f result_1 = MyMath::inverseMappingFunc(spherical_1);
    ASSERT_NEAR(cartesian_truth_1.x(), result_1.x(), 1e-3);
    ASSERT_NEAR(cartesian_truth_1.y(), result_1.y(), 1e-3);
    ASSERT_NEAR(cartesian_truth_1.z(), result_1.z(), 1e-3);

    Vector3f spherical_2= Vector3f( -M_PI/2, M_PI/3, 1);
    Vector3f cartesian_truth_2 = Vector3f(0,-sqrt(3)/2, 1./2);
    Vector3f result_2 = MyMath::inverseMappingFunc(spherical_2);
    ASSERT_NEAR(cartesian_truth_2.x(), result_2.x(), 1e-3);
    ASSERT_NEAR(cartesian_truth_2.y(), result_2.y(), 1e-3);
    ASSERT_NEAR(cartesian_truth_2.z(), result_2.z(), 1e-3);
 
    Vector3f spherical_3= Vector3f( 0, M_PI/2, 1);
    Vector3f cartesian_truth_3 = Vector3f(1,0,0);
    Vector3f result_3 = MyMath::inverseMappingFunc(spherical_3);
    ASSERT_NEAR(cartesian_truth_3.x(), result_3.x(), 1e-3);
    ASSERT_NEAR(cartesian_truth_3.y(), result_3.y(), 1e-3);
    ASSERT_NEAR(cartesian_truth_3.z(), result_3.z(), 1e-3);

    Vector3f spherical_4= Vector3f( 0, 2*M_PI/3, 1);
    Vector3f cartesian_truth_4 = Vector3f(sqrt(3)/2,0 ,-1./2);
    Vector3f result_4 = MyMath::inverseMappingFunc(spherical_4);
    ASSERT_NEAR(cartesian_truth_4.x(), result_4.x(), 1e-3);
    ASSERT_NEAR(cartesian_truth_4.y(), result_4.y(), 1e-3);
    ASSERT_NEAR(cartesian_truth_4.z(), result_4.z(), 1e-3);
 

    Vector3f spherical_5= Vector3f(M_PI/2, 2*M_PI/3, 1);
    Vector3f cartesian_truth_5 = Vector3f(0 ,sqrt(3)/2, -1./2);
    Vector3f result_5 = MyMath::inverseMappingFunc(spherical_5);
    ASSERT_NEAR(cartesian_truth_5.x(), result_5.x(), 1e-3);
    ASSERT_NEAR(cartesian_truth_5.y(), result_5.y(), 1e-3);
    ASSERT_NEAR(cartesian_truth_5.z(), result_5.z(), 1e-3);

  }
}




