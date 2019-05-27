#pragma once
#include <Eigen/Core>
namespace Loam{

  class ScanPoint{
    private:
      int m_index_ofSweep;//index of the belonging scan plane
      int m_index_inSweep;//positional index in the scan plane
      Eigen::Vector3f m_coords;
      float m_smoothness;
   
    public:
      ScanPoint():
        m_index_ofSweep( -1),
        m_index_inSweep( -1),
        m_coords(0.f,0.f,0.f),
        m_smoothness(-1.f){};

      ScanPoint(
          int t_index_of,
          int t_index_in,
          float t_x,
          float t_y,
          float t_z):
        m_index_ofSweep( t_index_of),
        m_index_inSweep( t_index_in),
        m_coords(t_x, t_y, t_z),
        m_smoothness(-1.f){};
 
      ScanPoint(
          int t_index_of,
          int t_index_in,
          Eigen::Vector3f t_coords):
        m_index_ofSweep( t_index_of),
        m_index_inSweep( t_index_in),
        m_coords(t_coords ),
        m_smoothness(-1.f){};
 
   
      ~ScanPoint() = default;

      inline void setSmoothnees( float t_smoothness){ m_smoothness = t_smoothness;};

      inline float getSmoothnees(){ return m_smoothness;};

      inline Eigen::Vector3f getCoords(){ return m_coords;};

      inline int getIndexOfSweep(){ return m_index_ofSweep;};

      inline int getIndexInSweep(){ return m_index_inSweep;};
  };
}




