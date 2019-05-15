#pragma once
#include <unistd.h>
#include <Eigen/Core>
#include "opencv2/opencv.hpp"



namespace Loam{

  typedef cv::Mat_< cv::Vec3b > RGBImage;

  class Drawer {

    private:
      double m_scale;
    public:
      Drawer(const double scale);

      void drawHollowPoligon(
          const RGBImage &t_drawing,
          const std::vector< cv::Point2d> &t_consecPoints,
          const cv::Scalar &t_color );

      void drawSegmentSeries(
          const RGBImage &t_drawing,
          const std::vector< cv::Point2d> &t_consecPoints,
          const cv::Scalar &t_color );


      void drawPatch(
          RGBImage &t_drawing,
          const cv::Point2d &t_point,
          const cv::Scalar &t_color);

      void drawLine(
          const RGBImage &t_drawing,
          const cv::Point2d &t_firstP,
          const cv::Point2d &t_secondP,
          const cv::Scalar &t_color);

      cv::Point2d convertFromWorldToImg(
          const RGBImage &t_drawing,
          const cv::Point2d &t_point);

      cv::Point2d bringPointInsideBoundaries(
          const RGBImage &t_drawing,
          const cv::Point2d &t_insidePoint,
          const cv::Point2d &t_outsidePoint);

      bool isInsideBoundaries(
          const RGBImage &t_drawing,
          const cv::Point2d &t_point);

      double crossProd2D(
          const Eigen::Vector2d &v1,
          const Eigen::Vector2d &v2);

      bool findIntersectionBetweenSegments(
          const cv::Point2d &p1,
          const cv::Point2d &p2,
          const cv::Point2d &q1,
          const cv::Point2d &q2,
          cv::Point2d &result);


  };
}
