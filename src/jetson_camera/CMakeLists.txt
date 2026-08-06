cmake_minimum_required(VERSION 3.16)
project(jetson_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

# GStreamer is found via pkg-config, not CMake config packages. gstreamer-app-1.0 comes
# from libgstreamer-plugins-base1.0-dev, which the base image does NOT ship -- it is
# installed in docker/Dockerfile. If this fails, that is why.
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)

add_executable(argus_camera_node src/argus_camera_node.cpp)
target_link_libraries(argus_camera_node PkgConfig::GST)
ament_target_dependencies(argus_camera_node
  rclcpp
  sensor_msgs
  image_transport
  camera_info_manager
)

install(TARGETS argus_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # No copyright headers yet, and this is not a released package.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
