add_executable(test_helloworld
  TestHelloWorld.cpp)

target_link_libraries(test_helloworld
  loam_library
  ${catkin_LIBRARIES}
  )

catkin_add_gtest(test_runner 
  TestDummy.cpp
  TestDatasetManager.cpp
  TestFeatureExtractor.cpp
  Starter.cpp
  )
target_link_libraries(test_runner
  loam_library
  srrg2_system_utils_library
  srrg2_boss_library
  ${catkin_LIBRARIES}
  )

add_executable(test_drawer
  TestDrawer.cpp
  )
target_link_libraries(test_drawer
  loam_library
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


add_executable(test_DEPRECATED
  TestDEPRECATED.cpp
  )
target_link_libraries(test_DEPRECATED
  loam_library
  srrg2_system_utils_library
  srrg2_boss_library
  ${catkin_LIBRARIES}
  )



