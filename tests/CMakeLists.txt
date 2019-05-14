add_executable(
  allTests
  Starter.cpp
  TestDummy.cpp
  TestDatasetManager.cpp
  )
target_link_libraries(
  allTests
  PUBLIC gtest_main
  PUBLIC gmock_main
  PUBLIC core
  PUBLIC ${OpenCV_LIBS}
  PUBLIC ${roscpp_LIBS}
  PUBLIC ${rosbag_LIBS}
  PUBLIC ${std_msgs_LIBS}
 )

add_test(
  NAME allTests
  COMMAND allTests
  )

