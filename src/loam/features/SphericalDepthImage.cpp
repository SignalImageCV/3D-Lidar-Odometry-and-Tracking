#include "SphericalDepthImage.hpp"

namespace Loam{
    
  Vector3f SphericalDepthImage::directMappingFunc(const Vector3f & t_cart_coords){
    return Vector3f(
        atan2( t_cart_coords.y(), t_cart_coords.x()),
        atan2( sqrt(pow(t_cart_coords.x(),2)+pow(t_cart_coords.y(),2)),
          t_cart_coords.z()),
        sqrt( pow(t_cart_coords.x(),2)+
          pow(t_cart_coords.y(),2)+
          pow( t_cart_coords.z(),2))
        );
  };

  Vector3f SphericalDepthImage::inverseMappingFunc(const Vector3f & t_spher_coords){
    float z = t_spher_coords.z() * cos( t_spher_coords.y());
    float proj = t_spher_coords.z() * sin( t_spher_coords.y());
    float x = proj * cos( t_spher_coords.x());
    float y = proj * sin( t_spher_coords.x());
    return Vector3f( x,y,z) ;
  };


}

