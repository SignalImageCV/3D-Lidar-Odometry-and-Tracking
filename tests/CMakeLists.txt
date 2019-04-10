add_executable(
  dummy
  Starter.cpp
  TestDummy.cpp
  )
target_link_libraries(
  dummy
  PUBLIC gtest_main
  PUBLIC gmock_main
  PUBLIC core
  PUBLIC ${OpenCV_LIBS}
 )

add_test(
  NAME dummy
  COMMAND dummy
  )

