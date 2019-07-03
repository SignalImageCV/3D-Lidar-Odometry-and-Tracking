#include "MyMath.hpp"
namespace Loam{

  //Deprecated because I am adopting eigen structures for the state
  // Eigen::Isometry2d MyMath::v2t(const std::vector<double> &t_vec){

  //   Eigen::Isometry2d T;
  //   T.setIdentity();
  //   Eigen::Vector2d transl( t_vec.at(0), t_vec.at(1));
  //   T.translation()= transl;
  //   double c = cos(t_vec.at(2));
  //   double s = sin(t_vec.at(2));
  //   T.linear() << c, -s, s, c;
  //   return T;
  // };

  Eigen::Isometry2d MyMath::v2t(const Eigen::Vector3d &t_vec){

    Eigen::Isometry2d T;
    T.setIdentity();
    Eigen::Vector2d transl( t_vec(0), t_vec(1));
    T.translation()= transl;
    double c = cos(t_vec(2));
    double s = sin(t_vec(2));
    T.linear() << c, -s, s, c;
    return T;
  };


  // std::vector<double> MyMath::t2v(const Eigen::Isometry2d& t_transf){

  //   Eigen::Vector3d v;
  //   v.head<2>()=t_transf.translation();
  //   v(2) = atan2(t_transf.linear()(1,0), t_transf.linear()(0,0));
  //   std::vector<double> vec = {v(0),v(1),v(2)};
  //   return vec;
  // };



  Eigen::Vector3d MyMath::t2v(const Eigen::Isometry2d& t_transf){

    Eigen::Vector3d v;
    v.head<2>()=t_transf.translation();
    v(2) = atan2(t_transf.linear()(1,0), t_transf.linear()(0,0));
    return v;
  };


  // void MyMath::rotate2D( std::vector<double> &t_point, const double t_angle_rad ){
  //   Eigen::Matrix2d R;
  //   R << cos(t_angle_rad ), -sin(t_angle_rad),
  //     sin(t_angle_rad), cos(t_angle_rad);
  //   Eigen::Vector2d p_old( t_point.at(0) , t_point.at(1));
  //   Eigen::Vector2d p_new = R* p_old;
  //   t_point.at(0) = p_new(0);
  //   t_point.at(1) = p_new(1);
  // };


  void MyMath::rotate2D( Eigen::Vector2d &t_point, const double t_angle_rad ){
    Eigen::Matrix2d R;
    R << cos(t_angle_rad ), -sin(t_angle_rad),
      sin(t_angle_rad), cos(t_angle_rad);
    t_point = R * t_point;
 };

  std::vector<double> MyMath::vecSum(const std::vector<double> &t_first,const std::vector<double> &t_second){
    std::vector<double> result;
    if ( t_first.size() == t_second.size() ){
      for (int i=0 ; i< t_first.size() ; ++i){
        result.push_back( t_first.at(i)+ t_second.at(i) );
      }
    }
    else{
      std::cout << "vecSum received vectors of different size, sum is not possible. \n";
      result = {0};
    }
    return result;
  };

  std::vector<double> MyMath::vecMultEleWise(const std::vector<double> &t_first,const std::vector<double> &t_second){
    std::vector<double> result;
    if ( t_first.size() == t_second.size() ){
      for (int i=0 ; i< t_first.size() ; ++i){
        result.push_back( t_first.at(i) * t_second.at(i) );
      }
    }
    else{
      std::cout << "vecSum received vectors of different size, sum is not possible. \n";
      result = {0};
    }
    return result;
  };


  double MyMath::boxMinusAngleRad(const double t_ref,const double t_actual) {
    Eigen::Matrix2d rot_ref;
    Eigen::Matrix2d rot_actual;
    rot_ref << cos( t_ref ), -sin(t_ref),
      sin(t_ref), cos(t_ref);
    rot_actual << cos( t_actual), -sin(t_actual),
      sin(t_actual), cos(t_actual);
    Eigen::Matrix2d rot = rot_actual.transpose() * rot_ref;
    return atan2( rot(1,0), rot(0,0));
  };

  double MyMath::boxPlusAngleRad(const double t_ref,const double t_actual) {
    Eigen::Matrix2d rot_ref;
    Eigen::Matrix2d rot_actual;
    rot_ref << cos( t_ref ), -sin(t_ref),
      sin(t_ref), cos(t_ref);
    rot_actual << cos( t_actual), -sin(t_actual),
      sin(t_actual), cos(t_actual);
    Eigen::Matrix2d rot = rot_actual * rot_ref;
    return atan2( rot(1,0), rot(0,0));
  };


  double MyMath::computeAvg(const double t_prevAvg,const int t_numEntries,const double t_newValue){
    if ( t_numEntries == 1){
      return t_newValue;
    }
    if ( t_numEntries == 0){
      std::cout << "Error because the num of entries to compute Avg should be always greater than zero, and in this case it is not";
      return -1;
    }
    return (( (t_numEntries -1) * t_prevAvg) + t_newValue) / (t_numEntries );
  };


  bool MyMath::checkIsInsideArcBoundaries( const float alpha, const float beta, const float theta, const float phi){
    return  ( alpha>0 and beta<0 and ( phi>alpha or phi<beta)) or ( phi>alpha and phi<beta and ( alpha<0 or beta>0));
  }

  Eigen::Vector3f MyMath::computeMuGaussian(const  std::vector< Eigen::Vector3f> & t_points){
    Eigen::Vector3f mu_partial = Eigen::Vector3f::Zero();
    for( auto & p: t_points){
      mu_partial += p;
    }
    return  mu_partial/ t_points.size();
  }

  Eigen::Matrix3f MyMath::computeSigmaGaussian(
        const  std::vector< Eigen::Vector3f> & t_points,
        const  Eigen::Vector3f & t_mu){
    Eigen::Matrix3f  sigma_partial = Eigen::Matrix3f::Zero();
    for ( auto & p: t_points){
      const Eigen::Vector3f vec = p - t_mu;
      sigma_partial += vec *  vec.transpose();
    }
    return sigma_partial/t_points.size(); 
  }



}
