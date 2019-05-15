#include "Drawer.hpp"
namespace Loam{

  Drawer::Drawer(const double scale):m_scale(scale){};


  void Drawer::drawHollowPoligon(
      const RGBImage &t_drawing,
      const std::vector< cv::Point2d> &t_consecPoints,
      const cv::Scalar &t_color){

    int poligon_edgesNum = t_consecPoints.size();

    for (int i = 0; i < poligon_edgesNum ; ++i) {
      if ( i == 0 ){
        drawLine( t_drawing,
            t_consecPoints.at(poligon_edgesNum -1),
            t_consecPoints.at(0),t_color);
      }
      else{
        drawLine( t_drawing,
            t_consecPoints.at(i-1),
            t_consecPoints.at(i),t_color);
      }
    }
  };

  void Drawer::drawSegmentSeries(
      const RGBImage &t_drawing,
      const std::vector< cv::Point2d> &t_consecPoints,
      const cv::Scalar &t_color){

    for (int i = 1; i < t_consecPoints.size() ; ++i) {
      drawLine( t_drawing,
          t_consecPoints.at(i-1),
          t_consecPoints.at(i),t_color);
    }
  };


  void Drawer::drawPatch(
      RGBImage &t_drawing,
      const cv::Point2d &t_point,
      const cv::Scalar &t_color){

    std::vector<cv::Point2d> points_imgFrame;
    cv::Point2d p_imgFrame =
      convertFromWorldToImg(t_drawing, t_point);

    points_imgFrame.push_back( p_imgFrame);

    cv::Point2d p_patchUp( p_imgFrame.x, p_imgFrame.y - 1);
    cv::Point2d p_patchDown( p_imgFrame.x, p_imgFrame.y + 1);
    cv::Point2d p_patchLeft( p_imgFrame.x -1, p_imgFrame.y );
    cv::Point2d p_patchRight( p_imgFrame.x +1, p_imgFrame.y );
    cv::Point2d p_patchLeftLeft( p_imgFrame.x -2, p_imgFrame.y );
    cv::Point2d p_patchLeftDown( p_imgFrame.x -1, p_imgFrame.y + 1);
    cv::Point2d p_patchDownDown( p_imgFrame.x, p_imgFrame.y +2 );
    cv::Point2d p_patchRightDown( p_imgFrame.x +1, p_imgFrame.y -1 );
    cv::Point2d p_patchRightRight( p_imgFrame.x +2, p_imgFrame.y );
    cv::Point2d p_patchRightUp( p_imgFrame.x +1, p_imgFrame.y - 1);
    cv::Point2d p_patchUpUp( p_imgFrame.x , p_imgFrame.y -2);
    cv::Point2d p_patchLeftUp( p_imgFrame.x -1, p_imgFrame.y -1);

    points_imgFrame.push_back( p_patchUp);
    points_imgFrame.push_back( p_patchDown);
    points_imgFrame.push_back( p_patchLeft);
    points_imgFrame.push_back( p_patchRight);
    points_imgFrame.push_back( p_patchLeftLeft);
    points_imgFrame.push_back( p_patchLeftDown);
    points_imgFrame.push_back( p_patchDownDown);
    points_imgFrame.push_back( p_patchRightDown);
    points_imgFrame.push_back( p_patchRightRight);
    points_imgFrame.push_back( p_patchRightUp);
    points_imgFrame.push_back( p_patchUpUp);
    points_imgFrame.push_back( p_patchLeftUp);

    for ( int i =0; i< points_imgFrame.size(); ++i){
      cv::Point2d p = points_imgFrame.at(i);
      if ( isInsideBoundaries(t_drawing, p)){
        t_drawing.at<cv::Vec3b>(p.y,p.x) =
          cv::Vec3b(t_color[0],t_color[1],t_color[2]);
      }
    }
  };

  void Drawer::drawLine(
      const RGBImage &t_drawing,
      const cv::Point2d &t_firstP,
      const cv::Point2d &t_secondP,
      const cv::Scalar &t_color){

    cv::Point2d firstP_imgFrame =
      convertFromWorldToImg(t_drawing, t_firstP);
    cv::Point2d secondP_imgFrame =
      convertFromWorldToImg(t_drawing, t_secondP);
    if ( isInsideBoundaries(t_drawing, firstP_imgFrame) &&
        isInsideBoundaries(t_drawing, secondP_imgFrame)){
      cv::line(t_drawing, firstP_imgFrame,
          secondP_imgFrame,t_color,2);
    }else{

      if ( isInsideBoundaries(t_drawing, firstP_imgFrame)){

        cv::Point2d secondP_worldFrameInside =
          bringPointInsideBoundaries(t_drawing,
              t_firstP, t_secondP);

        cv::Point2d secondP_imgFrameInside =
          convertFromWorldToImg( t_drawing,
              secondP_worldFrameInside);

        cv::line(t_drawing, firstP_imgFrame,
            secondP_imgFrameInside,t_color,2);

      }else{
        cv::Point2d firstP_worldFrameInside =
          bringPointInsideBoundaries(t_drawing,
              t_secondP, t_firstP);

        cv::Point2d firstP_imgFrameInside =
          convertFromWorldToImg( t_drawing,
              firstP_worldFrameInside);

        cv::line(t_drawing, firstP_imgFrameInside,
            secondP_imgFrame,t_color,2);
      }
    }
  }

  cv::Point2d Drawer::bringPointInsideBoundaries(
      const RGBImage &t_drawing,
      const cv::Point2d &t_insidePoint,
      const cv::Point2d &t_outsidePoint){

    double heigth = t_drawing.rows/m_scale;
    double width = t_drawing.cols/m_scale;

    cv::Point2d cornerDL( 0, 0); 
    cv::Point2d cornerUL( 0, heigth); 
    cv::Point2d cornerDR( width, 0); 
    cv::Point2d cornerUR( width, heigth); 

    cv::Point2d interPoint;

    if ( findIntersectionBetweenSegments(
          t_insidePoint, t_outsidePoint,
          cornerDL, cornerDR,
          interPoint)){
      return interPoint;
    }

    if ( findIntersectionBetweenSegments(
          t_insidePoint, t_outsidePoint,
          cornerUR, cornerDR,
          interPoint)){
      return interPoint;
    }

    if ( findIntersectionBetweenSegments(
          t_insidePoint, t_outsidePoint,
          cornerUL, cornerUR,
          interPoint)){
      return interPoint;
    }

    if ( findIntersectionBetweenSegments(
          t_insidePoint, t_outsidePoint,
          cornerUL, cornerDL,
          interPoint)){
      return interPoint;
    }
    return interPoint;
  }

  double Drawer::crossProd2D(
      const Eigen::Vector2d &v1,
      const Eigen::Vector2d &v2){

    return v1(0) * v2(1) - v1(1) * v2(0);
  }

  bool Drawer::findIntersectionBetweenSegments(
      const cv::Point2d &p1,
      const cv::Point2d &p2,
      const cv::Point2d &q1,
      const cv::Point2d &q2,
      cv::Point2d &result){

    Eigen::Vector2d p_1( p1.x, p1.y);
    Eigen::Vector2d p_2( p2.x, p2.y);
    Eigen::Vector2d q_1( q1.x, q1.y);
    Eigen::Vector2d q_2( q2.x, q2.y);

    Eigen::Vector2d r = p_2 -p_1;
    Eigen::Vector2d s = q_2 -q_1;

    double uNum = crossProd2D(q_1- p_1, r);
    double denom = crossProd2D( r, s);

    if (denom != 0) {

      double u= uNum/ denom;
      double t= crossProd2D(q_1 - p_1,s)/denom;
      if ( t>=0 && t<=1 && u>=0 && u<=1){

        Eigen::Vector2d crossPoint = p_1 + t*r;
        result.x = crossPoint(0);
        result.y = crossPoint(1);
        return true;
      }
      else{
        return false;
      }
    }else{
      return false;
    }
  }



  cv::Point2d Drawer::convertFromWorldToImg(
      const RGBImage &t_drawing,
      const cv::Point2d &t_point){

    cv::Point2d p_imgFrame( t_point.x * m_scale,
        t_drawing.rows - t_point.y * m_scale);
    return p_imgFrame;
  };

  bool Drawer::isInsideBoundaries(
      const RGBImage &t_drawing,
      const cv::Point2d &t_point){

    int heigth = t_drawing.rows;
    int width = t_drawing.cols;
    return t_point.x >= 0 && t_point.x < width &&
      t_point.y >= 0 && t_point.y < heigth;
  };

}
