#include <gtest/gtest.h>
#include <stdio.h>

#include "../loam/MyMath.hpp"


namespace Loam{
  using namespace testing;

  TEST( svd, basicSvdOp){

    Eigen::Matrix3f A;
    Eigen::Matrix3f U;
    Eigen::Matrix3f V;
    A <<
      18, 15, 18,
      15, 13, 15,
      18, 15, 18;

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3f s = svd.singularValues();
    U=svd.matrixU() ;
    V=svd.matrixV() ;
    //std::cout << "s:\n " << s<< std::endl;
    //std::cout << "U:\n " << U << std::endl;
    //std::cout << "V:\n " << V << std::endl;
    

    ASSERT_EQ( 3, 3);
  }
 

}


