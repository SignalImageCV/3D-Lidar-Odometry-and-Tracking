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
