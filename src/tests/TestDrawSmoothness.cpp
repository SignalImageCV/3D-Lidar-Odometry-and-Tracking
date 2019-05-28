#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/shell_colors.h>
#include <thread>
#include "../loam/DatasetManager.hpp"
#include "../loam/features/FeatureExtractor.hpp"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;

void drawingSmoothness( ViewerCanvasPtr canvas, const  string & filename){


  DatasetManager dM( filename);
  vector<ScanPoint> points;
  FeatureExtractor fE(0);

  std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >  colors;
  colors.resize(64);
  for(size_t i=0; i < colors.size(); ++i) {
    colors[i]= Vector3f(0.f, 2.f*float(i)/64, 2.f*(1.0 - float(i)/64));
  }
  

  while(ViewerCoreSharedQGL::isRunning()){

    points = dM.readMessageFromDataset();
    if(points.size() > 0){
      canvas->pushPointSize();
      canvas->setPointSize(3.0);

      fE.computeSmoothnessPaper( points);
      float min_smoothness = fE.findMinSmoothnessPoint(points).getSmoothness();
      float max_smoothness = fE.findMaxSmoothnessPoint(points).getSmoothness();
      double dim_interval= static_cast<double>( (max_smoothness - min_smoothness) / colors.size());
      
      cerr<<"Distribution : min="<<min_smoothness<<" max="<<max_smoothness<<" inter="<<dim_interval<<"\n";
      std::list<ScanPoint> ordered_for_smoothness=
        fE.sortForIncreasingSmoothness(points);


      std::vector<ScanPoint> curr_vec;
      int curr_interval_index = 0; 
      for( std::list<ScanPoint>::iterator it=ordered_for_smoothness.begin();
          it != ordered_for_smoothness.end(); ++it){

        int quotient= static_cast<int>( (it->getSmoothness() - min_smoothness)/dim_interval);
        if (  quotient<= curr_interval_index){
          curr_vec.push_back( *it);
        }
        else{
          cerr<<"num_index= "<<curr_interval_index<<" num_points: "<<curr_vec.size()<<"\n";
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
    canvas->flush();
    }
  }
}


int main( int argc, char** argv){

  string filename = "/home/dinies/gitrepos/loam/datasets/undulating_motion.boss";
  ViewerCoreSharedQGL viewer(argc, argv);
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingPoints_test");
  std::thread processing_thread(drawingSmoothness, canvas, filename);
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}

