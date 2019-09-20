#pragma once
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/shell_colors.h>
#include <thread>

#include "../loam/features/SphericalDepthImage.hpp"
#include "../loam/features/FeatureExtractor.hpp"
#include "../loam/DatasetManager.hpp"
#include "../loam/MyMath.hpp"

namespace Loam{
  class Visualizer {
    public:

      static void drawAxes(ViewerCanvasPtr canvas, const vector<PointNormalColor3fVectorCloud> & t_axes);

      static void drawNormals(ViewerCanvasPtr canvas, const PointNormalColor3fVectorCloud & t_points);

      static void visualizeSubrutine(ViewerCanvasPtr canvas, const std::string& filename);


      static void visualizeAxes( ViewerCanvasPtr canvas);

      static void visualizeCloud( ViewerCanvasPtr canvas,const PointNormalColor3fVectorCloud & t_cloud, const float t_points_size);

      static void visualizeLinesPlanes( ViewerCanvasPtr canvas);

      static void visualizeCondition( ViewerCanvasPtr canvas);

      static void visualizePointsWithNormals( ViewerCanvasPtr canvas);

      static void visualizeSphere(ViewerCanvasPtr canvas,const string & filename);

      static void visualizeCloudSmoothness( ViewerCanvasPtr canvas, const  string & filename);

      static void visualizeFullClouds( ViewerCanvasPtr canvas, const  string & filename);

      static void visualizeCleanedClouds(ViewerCanvasPtr first_canvas,
          ViewerCanvasPtr second_canvas, const  string & filename);


      static void visualizeCleanedWithNormals(ViewerCanvasPtr first_canvas,
          ViewerCanvasPtr second_canvas, const  string & filename);

      static void drawingSampledSmoothness( ViewerCanvasPtr canvas);

  };

}

  //  color3fRed
  //  color3fOrange
  //  color3fYellow
  //  color3fGreen
  //  color3fCyan
  //  color3fMagenta
  //  color3fViolet
  //  color3fBlue
  //  color3fBlack
  //  color3fWhite
  //  color3fDarkRed
  //  color3fDarkYellow
  //  color3fDarkGreen
  //  color3fDarkCyan
  //  color3fDarkMagenta
  //  color3fDarkViolet
  //  color3fDarkBlue
  //  color3fMint
  //  color3fLightGray
  //  color3fEerieBlack
  //  color3fPaleYellow
  //  color3fGunmetal

