#pragma once
#include <srrg_boss/id_context.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_boss/serializer.h>
#include <srrg_boss/deserializer.h>
#include <srrg_messages/instances.h>


namespace Loam{
  using namespace std;
  using namespace srrg2_core;


  class MatchableInterface{
    protected:
      Vector3f p_m;
      Matrix3f R_m;
      Matrix3f Omega_m;

    public:

      MatchableInterface() = default;

      ~MatchableInterface() = default;
  };

}


