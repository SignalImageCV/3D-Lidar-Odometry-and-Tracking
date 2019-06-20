#include "IntegralCell.hpp"

namespace Loam{
  IntegralCell::IntegralCell(
      const  Eigen::Vector3f & t_p_sum,
      const  Eigen::Matrix3f & t_p_quad,
      const int t_p_num):
    m_p_sum( t_p_sum),
    m_p_quad( t_p_quad),
    m_p_num( t_p_num)
  {};

  IntegralCell::IntegralCell():
    IntegralCell(
        Eigen::Vector3f::Zero(),
        Eigen::Matrix3f::Zero(),
        0)
  {}



  IntegralCell IntegralCell::operator+(IntegralCell & rhs){
    return IntegralCell( 
      m_p_sum + rhs.getPsum(),
      m_p_quad + rhs.getPquad(),
      m_p_num + rhs.getPnum()
      );
  };

  IntegralCell IntegralCell::operator-(IntegralCell & rhs){
    return IntegralCell( 
      m_p_sum - rhs.getPsum(),
      m_p_quad - rhs.getPquad(),
      m_p_num - rhs.getPnum()
      );
  }

  bool IntegralCell::operator==(IntegralCell& rhs){
    return this->getPsum() == rhs.getPsum() &&
      this->getPquad() == rhs.getPquad() &&
      this->getPnum() == rhs.getPnum();
  }

}


