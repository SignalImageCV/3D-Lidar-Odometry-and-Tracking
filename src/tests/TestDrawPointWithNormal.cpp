#include <srrg_system_utils/system_utils.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_messages/instances.h>



using namespace srrg2_core;
using namespace srrg2_qgl_viewport;


void visualizePointsWithNormals(ViewerCanvasPtr canvas);

void drawNormals(ViewerCanvasPtr canvas, const PointNormalColor3fVectorCloud & t_points);

int main( int argc, char** argv){

  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas = viewer.getCanvas("PointsWithNormals");

  std::thread processing_thread(
      visualizePointsWithNormals,canvas);

  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}



void visualizePointsWithNormals(ViewerCanvasPtr canvas){

  const int num_line_points = 10;
  PointNormalColor3fVectorCloud line_points;
  line_points.reserve(num_line_points);

  const Eigen::Vector3f line_start( 0, 0, 4); 
  const Eigen::Vector3f line_direction(0, 1, 0);

  const Eigen::Vector3f normal(1., 1., 0.);

  Eigen::Vector3f line_curr_point = line_start; 

  for (float i = 0; i < num_line_points; ++i) {
    PointNormalColor3f p;
    p.coordinates() = line_curr_point;
    p.color()= ColorPalette::color3fBlack();
    p.normal()= normal;
    line_points.push_back( p);
    line_curr_point+= line_direction;
  }
  while(  ViewerCoreSharedQGL::isRunning()){
    canvas->putPoints(line_points);
    drawNormals(canvas, line_points);
    canvas->flush();
  }
}


void drawNormals(ViewerCanvasPtr canvas, const PointNormalColor3fVectorCloud & t_points){

  std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >  colors;
  colors.resize(64);
  for(size_t i=0; i < colors.size(); ++i) {
    colors[i]= Vector3f( 2.f*float(i)/64, 2.f*(1.0 - float(i)/64), 0.f);
  }


  const int starting_color_index = 20;
  const int num_normal_points = 10;
  PointNormalColor3fVectorCloud normalPoints;
  normalPoints.reserve( t_points.size() * num_normal_points);
  for( auto & p: t_points){

    Eigen::Vector3f normal_vec = p.normal();

    Eigen::Vector3f normal_direction = Vector3f(normal_vec / num_normal_points);

    Eigen::Vector3f curr_normal_p= p.coordinates(); 
    for(int i = 0; i< num_normal_points; ++i){
      curr_normal_p += normal_direction;
      PointNormalColor3f new_point;
      new_point.coordinates() = curr_normal_p;
      new_point.color() = colors[i+ starting_color_index];
      normalPoints.push_back( new_point);
    }
  }
  canvas->putPoints( normalPoints);
}

