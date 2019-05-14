#include <gtest/gtest.h>

#include "../src/DatasetManager.hpp"

using namespace testing;
namespace Loam{

  TEST( datasetmanager , readdatafrombag){
    DatasetManager d = DatasetManager();

    d.readDataset();
  }
}


