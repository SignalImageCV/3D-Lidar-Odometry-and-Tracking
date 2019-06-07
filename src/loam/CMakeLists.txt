add_library( loam_library
  MyMath.cpp
  ScanPoint.cpp
  DatasetManager.cpp
  features/SphericalDepthImage.cpp
  features/FeatureExtractor.cpp
  )

target_link_libraries(loam_library
  PUBLIC ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  )

