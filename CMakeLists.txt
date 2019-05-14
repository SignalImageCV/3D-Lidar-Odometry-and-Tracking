CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

project(Loam CXX)

# Set the output directory for the build executables and libraries
ADD_DEFINITIONS(-DUNIX)
MESSAGE(STATUS "Compiling on Unix")

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -std=c++1z  -ggdb  -Wno-ignored-attributes ")

message("build type: '${CMAKE_BUILD_TYPE}'")
message("system type: '${CMAKE_SYSTEM_NAME}'")


set_property(GLOBAL PROPERTY GLOBAL_DEPENDS_DEBUG_MODE 1)

# Gtest START
# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()
# Gtest END


SET(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}")


# Find Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
# INCLUDE_DIRECTORIES(/usr/include/eigen3)

# Find Opencv
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})


# Find Roscpp
FIND_PACKAGE(roscpp REQUIRED)
INCLUDE_DIRECTORIES(${roscpp_INCLUDE_DIRS})


# Find Rosbag
FIND_PACKAGE(rosbag REQUIRED)
INCLUDE_DIRECTORIES(${rosbag_INCLUDE_DIRS})


# Find std_msgs
FIND_PACKAGE( std_msgs REQUIRED)
INCLUDE_DIRECTORIES(${std_msgs_INCLUDE_DIRS})


enable_testing()
add_subdirectory( src)
add_subdirectory( tests)







