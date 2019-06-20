#pragma once
#include <srrg_system_utils/system_utils.h>
#include "../MyMath.hpp"

using namespace srrg2_core;
namespace Loam{

  class IntegralCell {
    protected:
      Eigen::Vector3f  m_p_sum;
      Eigen::Matrix3f m_p_quad;
      int m_p_num;

    public:
      IntegralCell();

      IntegralCell(const  Eigen::Vector3f & t_p_sum,
          const  Eigen::Matrix3f & t_p_quad,
          const int t_p_num);



      IntegralCell operator+(IntegralCell & rhs);
      IntegralCell operator-(IntegralCell & rhs);

      bool operator==(IntegralCell& rhs);
      inline bool operator!=(IntegralCell& rhs){ return !(*this == rhs); }

      inline Eigen::Vector3f getPsum(){return m_p_sum;};
      inline Eigen::Matrix3f getPquad(){return m_p_quad;};
      inline int getPnum(){return m_p_num;};

  };
}
 
