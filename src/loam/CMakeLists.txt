add_library( loam_library
  Dummy.cpp
  Drawer.cpp
  ScanPoint.cpp
  DatasetManager.cpp
  features/FeatureExtractor.cpp
  )

target_link_libraries(loam_library
  PUBLIC ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  )

