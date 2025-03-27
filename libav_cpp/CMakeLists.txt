cmake_minimum_required(VERSION 3.8)
project(libav_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)

# libav_cpp as an interface library (i.e. no compilation, only headers)
add_library(
  libav_cpp SHARED
  src/libav_cpp.cpp)
target_include_directories(
  libav_cpp PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(libav_cpp avcodec avdevice avformat avutil swscale)

#
add_executable(
  decode_bag
  src/decode_bag.cpp)
target_include_directories(
  decode_bag PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(decode_bag libav_cpp)
ament_target_dependencies(decode_bag rclcpp rosbag2_storage rosbag2_transport sensor_msgs)

add_executable(
  decode_topic
  src/decode_topic.cpp)
target_include_directories(
  decode_topic PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(decode_topic libav_cpp)
ament_target_dependencies(decode_topic rclcpp sensor_msgs)

add_executable(
  stream_camera
  src/stream_camera.cpp)
target_include_directories(
  stream_camera PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(stream_camera libav_cpp)
ament_target_dependencies(stream_camera rclcpp sensor_msgs)

# install the library
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS libav_cpp
  EXPORT export_libav_cpp)
install(
  TARGETS decode_bag decode_topic stream_camera
  DESTINATION lib/${PROJECT_NAME})

# export the library for downstream projects
ament_export_include_directories(include)
ament_export_targets(export_libav_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
