#pragma once
#include <Eigen/Core>
#include "opencv2/opencv.hpp"


namespace Loam{

  typedef cv::Mat_< cv::Vec3b > RGBImage;

  typedef struct state_tag{
    Eigen::VectorXf mu;
    Eigen::MatrixXf sigma;

    state_tag( Eigen::VectorXf t_1,
               Eigen::MatrixXf t_2):
      mu( t_1),
      sigma( t_2)
    {}
    state_tag( Eigen::VectorXf t):
      mu( t)
    { sigma =  Eigen::MatrixXf::Zero(6,6); }
    state_tag( float t_x,float t_y,float t_z,
        float t_phi,float t_theta,float t_psi)
    { mu << t_x,t_y,t_z, t_phi,t_theta,t_psi;
      sigma =  Eigen::MatrixXf::Zero(6,6); }
    state_tag(){
      mu = Eigen::VectorXf::Zero(6);
      sigma =  Eigen::MatrixXf::Zero(6,6);;
    }
  } state;

  
}





