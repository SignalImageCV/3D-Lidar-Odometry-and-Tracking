#pragma once
#include <Eigen/Core>
#include "opencv2/opencv.hpp"


namespace Loam{

  typedef struct scanPoint_tag{
    int index_ofSweep; //at each sweep correspond a ring of points with index_inSweep
    int index_inSweep;
    Eigen::Vector3f coords;
    scanPoint_tag(int t_index_of,int t_index_in, float t_x, float t_y, float t_z):
      index_ofSweep( t_index_of),
      index_inSweep( t_index_in),
      coords(t_x, t_y, t_z)
    {}
    scanPoint_tag(int t_index_of,int t_index_in, Eigen::Vector3f t_coords):
      index_ofSweep( t_index_of),
      index_inSweep( t_index_in),
      coords(t_coords )
    {}
    scanPoint_tag():
      index_ofSweep( -1),
      index_inSweep( -1),
      coords(0.f,0.f,0.f)
    {}
  }scanPoint;

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





