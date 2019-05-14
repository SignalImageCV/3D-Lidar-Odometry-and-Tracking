add_library( loam_library SHARED
  Dummy.cpp
  )
target_link_libraries(loam_library
  ${catkin_LIBRARIES}
)

