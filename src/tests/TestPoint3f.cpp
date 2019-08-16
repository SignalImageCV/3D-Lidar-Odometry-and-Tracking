#pragma once
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include <gtest/gtest.h>
#include <stdio.h>


namespace Loam{
  using namespace testing;
  using namespace srrg2_core;

  TEST( Point3f, constructor ){
    Point3f  p;
    p.coordinates() = Vector3f( 2, 3,2);
    ASSERT_EQ( p.coordinates().y(), 3);
  }
  TEST( Point3f, constructorColorNormal ){
    PointNormalColor3f  p;
    p.coordinates() = Vector3f( 2, 3,2);
    p.color() = Vector3f( 1, 3,2);
    p.normal() =Vector3f( 4, 5,6);
    ASSERT_EQ( p.color().y(), 3);
    ASSERT_EQ( p.normal().y(), 5);
  }

  TEST( PointVec, vector){
    PointNormalColor3fVectorCloud points;
    points.resize(2);

    points[0].coordinates() = Vector3f( 1, 2,3);
    points[0].color() = Vector3f( 11,12,13);
    points[0].normal() = Vector3f( 21, 22,23);

    points[1].coordinates() = Vector3f( 4, 5,6);
    points[1].color() = Vector3f( 14,15,16);
    points[1].normal() = Vector3f( 24, 25,26);

    ASSERT_EQ( points[0].coordinates().y(), 2);
    ASSERT_EQ( points[0].color().y(), 12);
    ASSERT_EQ( points[0].normal().y(), 22);

    ASSERT_EQ( points[1].coordinates().y(), 5);
    ASSERT_EQ( points[1].color().y(), 15);
    ASSERT_EQ( points[1].normal().y(), 25);
  }

  TEST( PointVec, vectorCreateIncrementally){
    PointNormalColor3fVectorCloud points;
    points.reserve(3);

    ASSERT_EQ( points.size(), 0);
    for(unsigned int i=0; i < 3; ++i){
      PointNormalColor3f p;
      p.coordinates() = Vector3f( 1*i, 2*i, 3*i);
      p.color() = Vector3f( 11,12,13);
      p.normal() = Vector3f( 21, 22,23);
      points.push_back( p);
    }
    ASSERT_EQ( points.size(), 3);
    ASSERT_EQ( points[1].coordinates().y(), 2);
    ASSERT_EQ( points[1].color().y(), 12);
    ASSERT_EQ( points[1].normal().y(), 22);
 
  }

}


