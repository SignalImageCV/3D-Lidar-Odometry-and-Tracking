add_executable(test_helloworld
  TestHelloWorld.cpp)

target_link_libraries(test_helloworld
  loam_library
  ${catkin_LIBRARIES}
  )

catkin_add_gtest(test_runner 
  TestDummy.cpp
  Starter.cpp
  )
target_link_libraries(test_runner
  loam_library
  )

add_executable(test_drawer
  TestDrawer.cpp
  )
target_link_libraries(test_drawer
  loam_library
  ${catkin_LIBRARIES}
  )

add_executable(test_dataset
  TestDatasetManager.cpp
  )
target_link_libraries(test_dataset
  loam_library
  srrg2_system_utils_library
  srrg2_boss_library
  ${catkin_LIBRARIES}
  )



