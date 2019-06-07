#pragma once
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>




namespace Loam{
  using namespace std;
  using namespace srrg2_core;


   class SphericalDepthImage {

     public:
      SphericalDepthImage() = default;

      ~SphericalDepthImage() = default;
 
      //they maps a point from cartesian coords in spherical ones
      static  Vector3f directMappingFunc(const Vector3f & t_cart_coords);
      //  and from  spherical coords to cartesian ones
      static  Vector3f inverseMappingFunc(const Vector3f & t_spher_coords);





   };
}


