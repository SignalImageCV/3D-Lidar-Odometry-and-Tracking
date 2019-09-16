#pragma once

namespace Loam{

  typedef cv::Mat_< cv::Vec3b > RGBImage;

  typedef struct sphericalImage_params_tag{
     int num_vertical_rings;
     int num_points_ring;
     int epsilon_times;
     float epsilon_radius;
     float depth_differential_threshold;
     int min_neighboors_for_normal;
     int epsilon_c;
     float epsilon_d;
     float epsilon_n;
     float epsilon_l;
     float epsilon_dl;
     float epsilon_p;
     float epsilon_dp;
     sphericalImage_params_tag(
         const int t_num_vertical_rings,
         const int t_num_points_ring,
         const int t_epsilon_times,
         const float t_epsilon_radius,
         const float t_depth_differential_threshold,
         const int t_min_neighboors_for_normal,
         const int t_epsilon_c,
         const float t_epsilon_d,
         const float t_epsilon_n,
         const float t_epsilon_l,
         const float t_epsilon_dl,
         const float t_epsilon_p,
         const float t_epsilon_dp
         ):
       num_vertical_rings( t_num_vertical_rings),
       num_points_ring( t_num_points_ring),
       epsilon_times( t_epsilon_times),
       epsilon_radius( t_epsilon_radius),
       depth_differential_threshold(
           t_depth_differential_threshold),
       min_neighboors_for_normal(
           t_min_neighboors_for_normal),
       epsilon_c( t_epsilon_c),
       epsilon_d( t_epsilon_d),
       epsilon_n( t_epsilon_n),
       epsilon_l( t_epsilon_l),
       epsilon_dl( t_epsilon_dl),
       epsilon_p( t_epsilon_p),
       epsilon_dp( t_epsilon_dp)
    {}
     sphericalImage_params_tag():
       num_vertical_rings( -1),
       num_points_ring( -1),
       epsilon_times( -1),
       epsilon_radius( -1),
       depth_differential_threshold(-1),
       min_neighboors_for_normal(-1),
       epsilon_c(-1),
       epsilon_d(-1),
       epsilon_n(-1),
       epsilon_l(-1),
       epsilon_dl(-1),
       epsilon_p(-1),
       epsilon_dp(-1)
    {}
  }sphericalImage_params;

}

