cmake_minimum_required(VERSION 3.8)
project(deyes_capture_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS
  core
  imgproc
  calib3d
  aruco
  objdetect
  cudaimgproc
  cudawarping
  cudafilters
  cudastereo
  ximgproc
)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

add_executable(imx219_stereo_capture_node
  src/imx219_stereo_capture_node.cpp
)

add_executable(cuda_stereo_depth_node
  src/cuda_stereo_depth_node.cpp
)

add_executable(stereo_pointcloud_node
  src/stereo_pointcloud_node.cpp
)

target_include_directories(imx219_stereo_capture_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)

target_include_directories(cuda_stereo_depth_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)

target_include_directories(stereo_pointcloud_node PRIVATE
  include
)

target_link_directories(imx219_stereo_capture_node PRIVATE
  ${GSTREAMER_LIBRARY_DIRS}
)

target_link_directories(cuda_stereo_depth_node PRIVATE
  ${GSTREAMER_LIBRARY_DIRS}
)

target_link_libraries(imx219_stereo_capture_node
  ${OpenCV_LIBRARIES}
  yaml-cpp
  ${GSTREAMER_LIBRARIES}
)

target_link_libraries(cuda_stereo_depth_node
  ${OpenCV_LIBRARIES}
  yaml-cpp
  ${GSTREAMER_LIBRARIES}
)

target_compile_options(imx219_stereo_capture_node PRIVATE
  ${GSTREAMER_CFLAGS_OTHER}
)

target_compile_options(cuda_stereo_depth_node PRIVATE
  ${GSTREAMER_CFLAGS_OTHER}
)

ament_target_dependencies(imx219_stereo_capture_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
  std_msgs
)

ament_target_dependencies(cuda_stereo_depth_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(stereo_pointcloud_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
)

install(TARGETS
  imx219_stereo_capture_node
  cuda_stereo_depth_node
  stereo_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_projection test/test_depth_projection.cpp)
  if(TARGET test_depth_projection)
    target_include_directories(test_depth_projection PRIVATE include)
  endif()
  ament_add_gtest(test_capture_watchdog test/test_capture_watchdog.cpp)
  if(TARGET test_capture_watchdog)
    target_include_directories(test_capture_watchdog PRIVATE include)
  endif()
endif()

ament_package()
