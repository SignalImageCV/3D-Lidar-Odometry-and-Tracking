#include <gtest/gtest.h>
#include "../loam/features/SphericalDepthImage.hpp"

using namespace testing;
namespace Loam{
  TEST( SphericalDepthImage, directMapping){
    Vector3f cartesian_1 = Vector3f( -2, -2, 2);
    Vector3f spherical_truth_1 = Vector3f( -M_PI/2-M_PI/4, 0.955, sqrt(12));

    Vector3f cartesian_2 = Vector3f(0, -3, 3);
    Vector3f spherical_truth_2 = Vector3f( -M_PI/2, M_PI/4, sqrt(18));

    Vector3f result_1 = SphericalDepthImage::directMappingFunc(cartesian_1 );
    Vector3f result_2 = SphericalDepthImage::directMappingFunc(cartesian_2 );


    ASSERT_NEAR( spherical_truth_1.x(), result_1.x(), 1e-3);
    ASSERT_NEAR( spherical_truth_1.y(), result_1.y(), 1e-3);
    ASSERT_NEAR( spherical_truth_1.z(), result_1.z(), 1e-3);

    ASSERT_NEAR( spherical_truth_2.x(), result_2.x(), 1e-3);
    ASSERT_NEAR( spherical_truth_2.y(), result_2.y(), 1e-3);
    ASSERT_NEAR( spherical_truth_2.z(), result_2.z(), 1e-3);

 }

  TEST( SphericalDepthImage, inverseMapping){
    Vector3f spherical_1= Vector3f( -M_PI/2-M_PI/4, 0.955, sqrt(12));
    Vector3f cartesian_truth_1 = Vector3f( -2, -2, 2);

    Vector3f spherical_2= Vector3f( -M_PI/2, M_PI/4, sqrt(18));
    Vector3f cartesian_truth_2 = Vector3f(0, -3, 3);

    Vector3f result_1 = SphericalDepthImage::inverseMappingFunc(spherical_1);
    Vector3f result_2 = SphericalDepthImage::inverseMappingFunc(spherical_2);

    ASSERT_NEAR(cartesian_truth_1.x(), result_1.x(), 1e-3);
    ASSERT_NEAR(cartesian_truth_1.y(), result_1.y(), 1e-3);
    ASSERT_NEAR(cartesian_truth_1.z(), result_1.z(), 1e-3);

    ASSERT_NEAR(cartesian_truth_2.x(), result_2.x(), 1e-3);
    ASSERT_NEAR(cartesian_truth_2.y(), result_2.y(), 1e-3);
    ASSERT_NEAR(cartesian_truth_2.z(), result_2.z(), 1e-3);
 
  }

}

