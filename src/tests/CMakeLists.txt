catkin_add_gtest(test_runner 
  TestSphericalDepthImage.cpp
  TestIntegralImage.cpp
  TestPoint3f.cpp
  TestDataPoint.cpp
  TestMatchable.cpp
  TestEigenSvd.cpp
  TestMyMath.cpp
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


add_executable(test_customConfigurableExec
  TestCustomConfigurableExec.cpp
  )
target_link_libraries(test_customConfigurableExec
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

add_executable(test_selfMatchablesCorrespondenceFinder
  TestSelfMatchablesCorrespondenceFinder.cpp
  )
target_link_libraries(test_selfMatchablesCorrespondenceFinder
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

add_executable(test_correspondanceFinder
  TestCorrespondanceFinder.cpp
  )
target_link_libraries(test_correspondanceFinder
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



add_executable(test_selfMatchablesSolver
  TestSelfMatchablesSolver.cpp
  )
target_link_libraries(test_selfMatchablesSolver
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

add_executable(test_solver
  TestSolver.cpp
  )
target_link_libraries(test_solver
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



add_executable(test_aligner
  TestAligner.cpp
  )
target_link_libraries(test_aligner
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


add_executable(test_tracker
  TestTracker.cpp
  )
target_link_libraries(test_tracker
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

add_executable(test_drawPlane
  TestDrawPlane.cpp
  )
target_link_libraries(test_drawPlane
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )


add_executable(test_drawNormalOfLine
  TestDrawNormalOfLine.cpp
  )
target_link_libraries(test_drawNormalOfLine
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

add_executable(test_draw2DclustererImgs
  TestDraw2DclustererImgs.cpp
  )
target_link_libraries(test_draw2DclustererImgs
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  )


add_executable(test_draw2DrealData
  TestDraw2DrealData.cpp
  )
target_link_libraries(test_draw2DrealData
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


add_executable(test_drawClusters
  TestDrawClusters.cpp
  )
target_link_libraries(test_drawClusters
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  pthread
  )

add_executable(test_drawMatchablesDummyData
  TestDrawMatchablesDummyData.cpp
  )
target_link_libraries(test_drawMatchablesDummyData
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )


add_executable(test_drawMatchablesRealData
  TestDrawMatchablesRealData.cpp
  )
target_link_libraries(test_drawMatchablesRealData
  loam_library
  srrg2_system_utils_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )


add_executable(test_tuningParamsForMatchables
  TestTuningParamsForMatchables.cpp
  )
target_link_libraries(test_tuningParamsForMatchables
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

