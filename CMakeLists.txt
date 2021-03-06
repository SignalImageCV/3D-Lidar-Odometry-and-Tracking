cmake_minimum_required(VERSION 2.8.3)
project(loam)

find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg2_core
  srrg2_core_ros
  srrg2_solver
  srrg2_slam_interfaces
  srrg2_qgl_viewport
 )

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)

find_package(Eigen3 REQUIRED)
find_package(QGLViewer REQUIRED)

find_package(Cholmod REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "${PROJECT_NAME}|OpenCV version ${OpenCV_VERSION}")
message(STATUS "${PROJECT_NAME}|OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    loam_library
  CATKIN_DEPENDS
    srrg_cmake_modules
    srrg2_core
    srrg2_core_ros
    srrg2_solver
    srrg2_slam_interfaces
    srrg2_qgl_viewport
    srrg2_shaslam
  DEPENDS
    OpenCV
    SRRG_QT
    QGLViewer
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${SRRG_QT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

link_directories(${catkin_LIBRARY_DIRS})

# set sources
add_subdirectory(src)
