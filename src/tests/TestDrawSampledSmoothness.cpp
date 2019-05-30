#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/shell_colors.h>
#include <thread>
#include "../loam/features/FeatureExtractor.hpp"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;

void drawingSampledSmoothness( ViewerCanvasPtr canvas){

  Eigen::Vector3f center( 5, 5, 5);
  float length_edge = 6;
  int precision = 5;
   
  std::vector<ScanPoint> cube_points =
    ScanPoint::generateCubeSampledScanPoints(center, length_edge, precision);


  int num_points_axes = 100;

  Point3fVectorCloud pointcloud_x_axis;
  Point3fVectorCloud pointcloud_y_axis;
  Point3fVectorCloud pointcloud_z_axis;
  pointcloud_x_axis.resize( num_points_axes);
  pointcloud_y_axis.resize( num_points_axes);
  pointcloud_z_axis.resize( num_points_axes);

  float x = 0;
  float y = 0;
  float z = 0;
    
  for (unsigned int i = 0; i < pointcloud_x_axis.size(); ++i) {
    x += 0.2;
    y += 0.2;
    z += 0.2;
    pointcloud_x_axis[i].coordinates()=Vector3f( x,0,0);
    pointcloud_y_axis[i].coordinates()=Vector3f( 0,y,0);
    pointcloud_z_axis[i].coordinates()=Vector3f( 0,0,z);
  }

  const int num_line_points = 50;
  vector<ScanPoint> line_points;
  line_points.reserve(num_line_points);

  const Eigen::Vector3f line_start( 7, 7, -3); 
  Eigen::Vector3f line_curr_point = line_start; 
  const Eigen::Vector3f line_direction(-0.1, -0.1, 0.4);

  for (unsigned int i = 0; i < num_line_points; ++i) {
    ScanPoint p( 0, i, line_curr_point);
    line_points.push_back( p);
    line_curr_point+= line_direction;
  }

  Point3fVectorCloud pointcloud_line;
  pointcloud_line.resize( num_line_points);
  for (unsigned int j = 0; j < pointcloud_line.size(); ++j) {
    pointcloud_line[j].coordinates()=line_points[j].getCoords();
  }
          

  FeatureExtractor fE(0);

  std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >  colors;
  colors.resize(64);
  for(size_t i=0; i < colors.size(); ++i) {
    colors[i]= Vector3f( 2.f*float(i)/64, 2.f*(1.0 - float(i)/64), 0.f);
  }

  for( auto & p: cube_points){
    //std::cerr<<"coords: "<<p.getCoords()<<"\n";
    //std::cerr<<"bef: "<<p.getSmoothness()<<"\n";
    fE.computeSingleSmoothnessPaper(line_points, p );
    //std::cerr<<"aft: "<<p.getSmoothness()<<"\n";
  }
  while(ViewerCoreSharedQGL::isRunning()){
    canvas->pushPointSize();
    canvas->setPointSize(7.0);
    float min_smoothness = fE.findMinSmoothnessPoint(cube_points).getSmoothness();
    float max_smoothness = fE.findMaxSmoothnessPoint(cube_points).getSmoothness();
    double dim_interval= static_cast<double>( (max_smoothness - min_smoothness) / colors.size());

    std::list<ScanPoint> ordered_for_smoothness= fE.sortForIncreasingSmoothness(cube_points);

    //cerr<<"Distribution:min="<<min_smoothness<<" max="<<max_smoothness<<" inter="<<dim_interval<<"\n";

    std::vector<ScanPoint> curr_vec;
    int curr_interval_index = 0; 
    for( std::list<ScanPoint>::iterator it=ordered_for_smoothness.begin();
      it != ordered_for_smoothness.end(); ++it){

      int quotient= static_cast<int>( (it->getSmoothness() - min_smoothness)/dim_interval);
      if (  quotient<= curr_interval_index){
        curr_vec.push_back( *it);
      }
      else{
        //cerr<<"num_index= "<<curr_interval_index<<" num_points: "<<curr_vec.size()<<"\n";
        curr_interval_index = quotient;
        if( curr_vec.size() > 0){
          Point3fVectorCloud pointcloud;
          pointcloud.resize( curr_vec.size());
          for (unsigned int j = 0; j < pointcloud.size(); ++j) {
            pointcloud[j].coordinates()=curr_vec[j].getCoords();
          }
          canvas->pushColor();
          canvas->setColor(colors[curr_interval_index]);
          canvas->putPoints( pointcloud);
          canvas->popAttribute();
          curr_vec.clear();
          curr_vec.push_back(*it);
        }
      }
    }
    canvas->popAttribute();

    canvas->pushPointSize();
    canvas->setPointSize(3.0);
    //x_axis
    canvas->pushColor();
    canvas->setColor(ColorPalette::color3fOrange());
    canvas->putPoints( pointcloud_x_axis);
    canvas->popAttribute();
    //y_axis
    canvas->pushColor();
    canvas->setColor(ColorPalette::color3fGreen());
    canvas->putPoints( pointcloud_y_axis);
    canvas->popAttribute();
    //z_axis
    canvas->pushColor();
    canvas->setColor(ColorPalette::color3fBlue());
    canvas->putPoints( pointcloud_z_axis);
    canvas->popAttribute();
    //line
    canvas->pushColor();
    canvas->setColor( Vector3f(0.f,0.5f,0.f));
    canvas->putPoints( pointcloud_line);
    canvas->popAttribute();

    canvas->popAttribute();
    canvas->flush();
  }
}


int main( int argc, char** argv){

  ViewerCoreSharedQGL viewer(argc, argv);
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingSampleSmoothness");
  std::thread processing_thread(drawingSampledSmoothness, canvas );
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}

