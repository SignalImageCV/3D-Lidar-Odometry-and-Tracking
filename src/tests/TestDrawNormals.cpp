#include <srrg_system_utils/parse_command_line.h>

#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/shell_colors.h>
#include <thread>

#include "loam/features/SphericalDepthImage.hpp"
#include "loam/DatasetManager.hpp"
#include "loam/MyMath.hpp"


using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_qgl_viewport;
using namespace Loam;

const char* banner[] = {
    "dynamic executor",
      0
};

void visualizeCleanedWithNormals( ViewerCanvasPtr first_canvas, ViewerCanvasPtr second_canvas, const  string & filename);

int main( int argc, char** argv){

  ParseCommandLine cmd_line(argv,banner);
  ArgumentString  dataset (&cmd_line, "d", "dataset", "path to dataset" , "");
  cmd_line.parse();

  messages_registerTypes();
  srrgInit( argc, argv, "hi");

  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas1 = viewer.getCanvas("noFlatPlusnormals");
  ViewerCanvasPtr canvas2 = viewer.getCanvas("rawScene");

  std::thread processing_thread(
      visualizeCleanedWithNormals,canvas1,canvas2,dataset.value());

  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}


  void visualizeCleanedWithNormals(ViewerCanvasPtr first_canvas,
          ViewerCanvasPtr second_canvas, const  string & filename){

    const sphericalImage_params params(
      60, //num_vertical_rings
      100, //num_points_ring
      10, //epsilon_times
      0.15, //epsilon_radius
      0.1, //depth_differential_threshold
      8,  //min_neighboors_for_normal
      5, //epsilon_c
      0.1, //epsilon_d
      0.02, //epsilon_n
      1, //epsilon_l
      1, //epsilon_dl
      1, //epsilon_p
      1 //epsilon_dp
    );
        
    SphericalDepthImage sph_Image;
    DatasetManager dM( filename);

    while( ViewerCoreSharedQGL::isRunning()){

      PointNormalColor3fVectorCloud current_point_cloud= dM.readMessageFromDataset();
      if( current_point_cloud.size()> 0){

 
        sph_Image= SphericalDepthImage(current_point_cloud,params);

        sph_Image.initializeIndexImage();
        PointNormalColor3fVectorCloud unprojected_cloud= sph_Image.getPointCloud();
        sph_Image.removeFlatSurfaces();
        sph_Image.collectNormals();
        PointNormalColor3fVectorCloud resulting_cloud= sph_Image.getPointCloud();

        first_canvas->putPoints(resulting_cloud);
        //Visualizer::drawNormals( first_canvas,  resulting_cloud);

        second_canvas->putPoints(unprojected_cloud);

        first_canvas->flush();
        second_canvas->flush();
      }
    }
  }


