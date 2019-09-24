#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>

#include <srrg_viewer/viewer_core/color_palette.h>

using namespace std;
using namespace srrg2_core;

namespace Loam{
  class Drawer{
    public:
      static vector<PointNormalColor3fVectorCloud> createAxes();

      static PointNormalColor3fVectorCloud createCircle(const float radius);

      static PointNormalColor3fVectorCloud createLine(
          const Vector3f & center_point,
          const Vector3f & direction,
          const float length,
          const float precision,
          const Vector3f & color= Vector3f( 0.,0.,0.)
          );

      static PointNormalColor3fVectorCloud createPlane(
          const Vector3f & center_point,
          const Vector3f & first_direction,
          const Vector3f & second_direction,
          const float length_firstDir,
          const float length_secondDir,
          const float precision_firstDir,
          const float precision_secondDir,
          const Vector3f & color= Vector3f( 0.,0.,0.)
          );

  };
}


