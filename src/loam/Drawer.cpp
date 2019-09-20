#include "Drawer.hpp"

using namespace std;

namespace Loam{

  vector<PointNormalColor3fVectorCloud> Drawer::createAxes(){

    vector< PointNormalColor3fVectorCloud> axes;
    
    int num_points = 100;
    PointNormalColor3fVectorCloud pointcloud_x_axis;
    PointNormalColor3fVectorCloud pointcloud_y_axis;
    PointNormalColor3fVectorCloud pointcloud_z_axis;

    pointcloud_x_axis.reserve( num_points);
    pointcloud_y_axis.reserve( num_points);
    pointcloud_z_axis.reserve( num_points);

    float x = 0;
    float y = 0;
    float z = 0;
    for (unsigned int i = 0; i < num_points; ++i) {
      x += 0.1;
      y += 0.1;
      z += 0.1;
      PointNormalColor3f p_x;
      PointNormalColor3f p_y;
      PointNormalColor3f p_z;

      p_x.coordinates()=Vector3f( x,0,0);
      p_x.color()= ColorPalette::color3fDarkCyan();
      p_y.coordinates()=Vector3f( 0,y,0);
      p_y.color()= ColorPalette::color3fDarkGreen();
      p_z.coordinates()=Vector3f( 0,0,z);
      p_z.color()= ColorPalette::color3fDarkRed();

      pointcloud_x_axis.push_back( p_x);
      pointcloud_y_axis.push_back( p_y);
      pointcloud_z_axis.push_back( p_z);

   }
    axes.push_back(pointcloud_x_axis);
    axes.push_back(pointcloud_y_axis);
    axes.push_back(pointcloud_z_axis);
    return axes;
  }

  PointNormalColor3fVectorCloud Drawer::createCircle(const float radius = 1.){

    PointNormalColor3fVectorCloud circle_point_cloud;
    circle_point_cloud.reserve(360);
    
    float j = 0;float k = 0; const float l = 0;
    vector<Vector3f> circle_points;
    circle_points.reserve(360);
    for( float angle = 0; angle <= 2*M_PI; angle+= M_PI/180){
      j = radius* cos( angle); 
      k = radius* sin( angle); 
      PointNormalColor3f p;
      p.coordinates() = Vector3f( j, k, l);
      p.color() = ColorPalette::color3fBlack();
      circle_point_cloud.push_back( p);
    }

    return circle_point_cloud;
  }

  PointNormalColor3fVectorCloud Drawer::createLine(
      const Vector3f & center_point,
      const Vector3f & direction,
      const float length,
      const float precision){

    const int num_points = length/precision;
    PointNormalColor3fVectorCloud linePoints;
    linePoints.reserve( num_points);

    Vector3f normalized_direction = direction/ direction.norm();

    const Vector3f right_direction  = normalized_direction;
    const Vector3f left_direction  = -normalized_direction;

    Vector3f left_edge_coords = center_point+ left_direction*precision/2;
    Vector3f right_edge_coords = center_point+ right_direction*precision/2;

    for ( unsigned int i=0; i<num_points/2; ++i){

      PointNormalColor3f left_edge_point;
      PointNormalColor3f right_edge_point;

      left_edge_point.coordinates() = left_edge_coords;
      left_edge_point.color() = ColorPalette::color3fBlack();
      right_edge_point.coordinates() = right_edge_coords;
      right_edge_point.color() = ColorPalette::color3fBlack();

      linePoints.push_back( left_edge_point);
      linePoints.push_back( right_edge_point);

      left_edge_coords += left_direction* precision;
      right_edge_coords += right_direction* precision;


    }

    return linePoints;
  }

  PointNormalColor3fVectorCloud Drawer::createPlane(
      const Vector3f & center_point,
      const Vector3f & first_direction,
      const Vector3f & second_direction,
      const float length_firstDir,
      const float length_secondDir,
      const float precision_firstDir,
      const float precision_secondDir){

    const int num_points_firstDir = length_firstDir/ precision_firstDir;
    const int num_points_secondDir = length_secondDir/ precision_secondDir;
    const int tot_num_points = num_points_firstDir * num_points_secondDir;

    PointNormalColor3fVectorCloud planePoints;
    planePoints.reserve( tot_num_points);

    Vector3f normalized_direction = first_direction/ first_direction.norm();

    const Vector3f right_direction  = normalized_direction;
    const Vector3f left_direction  = -normalized_direction;

    Vector3f left_edge_coords = center_point+ left_direction*precision_firstDir/2;
    Vector3f right_edge_coords = center_point+ right_direction*precision_firstDir/2;

    for ( unsigned int i=0; i<num_points_firstDir/2; ++i){

      PointNormalColor3fVectorCloud left_line = Drawer::createLine(
          left_edge_coords, second_direction, length_secondDir, precision_secondDir);

      PointNormalColor3fVectorCloud right_line = Drawer::createLine(
          right_edge_coords, second_direction, length_secondDir, precision_secondDir);

      planePoints.insert(
          planePoints.end(),
          std::make_move_iterator( left_line.begin()),
          std::make_move_iterator( left_line.end())
          );

      planePoints.insert(
          planePoints.end(),
          std::make_move_iterator( right_line.begin()),
          std::make_move_iterator( right_line.end())
          );
      left_edge_coords += left_direction* precision_firstDir;
      right_edge_coords += right_direction* precision_firstDir;
    }
    return planePoints;
  }
}

