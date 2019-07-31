catkin_add_gtest(test_runner 
  TestDatasetManager.cpp
  TestSphericalDepthImage.cpp
  TestIntegralImage.cpp
  TestPoint3f.cpp
  TestDataPoint.cpp
  TestMatchable.cpp
  TestEigenSvd.cpp
  Starter.cpp
  )

target_link_libraries(test_runner
  loam_library
  srrg2_system_utils_library
  srrg2_boss_library
  ${SRRG_QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )


add_executable(test_viewer
  TestViewer.cpp
  )
target_link_libraries(test_viewer
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  srrg2_boss_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )



add_executable(test_drawAxes
  TestDrawAxes.cpp
  )
target_link_libraries(test_drawAxes
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )

add_executable(test_drawLinesPlanes
  TestDrawLinesPlanes.cpp
  )
target_link_libraries(test_drawLinesPlanes
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )

add_executable(test_draw2DindexImg
  TestDraw2DindexImg.cpp
  )
target_link_libraries(test_draw2DindexImg
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  )

add_executable(test_draw2Dnormals
  TestDraw2Dnormals.cpp
  )
target_link_libraries(test_draw2Dnormals
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  )

add_executable(test_drawPathImage
  TestDrawPathImage.cpp
  )
target_link_libraries(test_drawPathImage
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  )





add_executable(test_fullCloud
  TestDrawFullClouds.cpp
  )
target_link_libraries(test_fullCloud
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )



add_executable(test_visualizeCloud
  TestCloudVisualization.cpp
  )
target_link_libraries(test_visualizeCloud
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )

add_executable(test_boolCondition
  TestBooleanCondition.cpp
  )
target_link_libraries(test_boolCondition
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )

add_executable(test_drawSampledSmoothness
  TestDrawSampledSmoothness.cpp
  )
target_link_libraries(test_drawSampledSmoothness
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )

add_executable(test_drawSphere
  TestDrawSphere.cpp
  )
target_link_libraries(test_drawSphere
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )

add_executable(test_drawVerticalPoints
  TestDrawVerticalPoints.cpp
  )
target_link_libraries(test_drawVerticalPoints
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )

add_executable(test_drawNormals
  TestDrawNormals.cpp
  )
target_link_libraries(test_drawNormals
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )

add_executable(test_drawPointWithNormal
  TestDrawPointWithNormal.cpp
  )
target_link_libraries(test_drawPointWithNormal
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )

