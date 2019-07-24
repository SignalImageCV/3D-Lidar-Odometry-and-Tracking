add_library( loam_library
  MyMath.cpp
  ScanPoint.cpp
  DatasetManager.cpp
  Visualizer.cpp
  features/SphericalDepthImage.cpp
  features/Clusterer.cpp
  features/FeatureExtractor.cpp
  features/IntegralImage.cpp
  features/IntegralCell.cpp
  features/DataPoint.cpp
  features/Matchable.cpp
  features/Point.cpp
  features/Plane.cpp
  features/Line.cpp
  )

target_link_libraries(loam_library
  PUBLIC ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  )

