add_library( core
  Dummy.cpp
  )

target_include_directories(
  core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
  )
