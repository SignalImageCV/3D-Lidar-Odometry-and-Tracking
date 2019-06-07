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
  TEST( Point3f, constructorColor ){
    PointNormalColor3f  p;
    p.coordinates() = Vector3f( 2, 3,2);
    p.color() = Vector3f( 2, 3,2);
    ASSERT_EQ( p.color().y(), 3);
  }

}


