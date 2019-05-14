#include <gtest/gtest.h>
#include "../loam/Dummy.hpp"

using namespace testing;
namespace Loam{

  TEST( dummy , simple){
    Dummy d(4);
    d.printNumCores();
  }
}


