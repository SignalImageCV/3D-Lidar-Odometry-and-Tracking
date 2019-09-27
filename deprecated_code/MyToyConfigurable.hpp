#pragma once
#include <srrg_config/property_configurable.h>

//todo remove this stub class: used to understand the srrg2 pipeline
namespace Loam{
  using namespace srrg2_core;


  class MyToyConfigurable: public Configurable {
  public:
    PARAM(PropertyFloat, known_num, "num to obtain from the module", 7.9, 0);

    float spitOut();
  };

  using MyToyConfigurablePtr = std::shared_ptr<MyToyConfigurable>;
}
