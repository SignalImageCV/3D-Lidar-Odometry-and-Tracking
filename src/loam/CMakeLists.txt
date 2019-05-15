add_library( loam_library
  Dummy.cpp
  Drawer.cpp
  DatasetManager.cpp
  )

target_link_libraries(loam_library
  PUBLIC ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  )

