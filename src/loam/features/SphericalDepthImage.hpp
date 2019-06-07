#pragma once


namespace Loam{

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


