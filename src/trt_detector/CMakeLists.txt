cmake_minimum_required(VERSION 3.16)
project(trt_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# Postprocessing (decode + NMS) is a plain library with no TensorRT or CUDA dependency,
# so it builds and its tests run anywhere -- including CI runners with no GPU.
add_library(postprocess src/postprocess.cpp)
target_include_directories(postprocess PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(postprocess ${OpenCV_LIBS})

# TensorRT is NOT installed in the container image -- the NVIDIA runtime mounts the
# host's libraries in at run (and build) time. On CI runners it is absent, so the node
# itself is skipped there while the library and tests still build.
find_library(NVINFER_LIB nvinfer PATHS /usr/lib/aarch64-linux-gnu)
if(NVINFER_LIB)
  find_package(CUDA REQUIRED)
  add_executable(trt_detector_node src/trt_detector_node.cpp)
  target_include_directories(trt_detector_node PRIVATE
    /usr/include/aarch64-linux-gnu        # NvInfer.h (runtime-mounted)
    ${CUDA_INCLUDE_DIRS})
  target_link_libraries(trt_detector_node
    postprocess
    ${NVINFER_LIB}
    ${CUDA_CUDART_LIBRARY})
  ament_target_dependencies(trt_detector_node rclcpp sensor_msgs vision_msgs std_msgs)
  install(TARGETS trt_detector_node DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING
    "TensorRT not found: building the postprocess library and tests only. "
    "This is expected on CI; on the Jetson it means the container is running "
    "without the NVIDIA runtime.")
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS postprocess
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_postprocess test/test_postprocess.cpp)
  target_link_libraries(test_postprocess postprocess)
endif()

ament_package()
