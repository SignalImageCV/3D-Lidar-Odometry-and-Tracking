add_executable(test_helloworld
  TestHelloWorld.cpp)

target_link_libraries(test_helloworld
  loam_library
  ${catkin_LIBRARIES}
  )

