add_library( loam_library
  MyMath.cpp
  ScanPoint.cpp
  DatasetManager.cpp
  Visualizer.cpp
  features/SphericalDepthImage.cpp
  features/FeatureExtractor.cpp
  features/IntegralImage.cpp
  features/DataPoint.cpp
  )

target_link_libraries(loam_library
  PUBLIC ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  )

