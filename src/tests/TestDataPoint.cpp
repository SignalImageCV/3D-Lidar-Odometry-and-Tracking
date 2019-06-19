#include <gtest/gtest.h>
#include <stdio.h>

#include "../loam/features/DataPoint.hpp" 

namespace Loam{
  using namespace testing;

  TEST( DataPoint, constructorNoArgs){
    DataPoint dP_empty = DataPoint();
    ASSERT_EQ( dP_empty.getIndexContainer(),-1);
    ASSERT_FALSE( dP_empty.getIsVertical());
    ASSERT_FALSE( dP_empty.getIsTaken());
  }
}


