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
find_package(CUDA REQUIRED)

# TensorRT is NOT installed in the image -- the NVIDIA container runtime mounts the
# host's libraries and headers in at run (and build) time. This target therefore only
# builds inside a container started with the nvidia runtime; that is already this
# project's default. If find_library fails here, you are building without it.
find_library(NVINFER_LIB nvinfer PATHS /usr/lib/aarch64-linux-gnu REQUIRED)

add_executable(trt_detector_node src/trt_detector_node.cpp)
target_include_directories(trt_detector_node PRIVATE
  /usr/include/aarch64-linux-gnu        # NvInfer.h (runtime-mounted)
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(trt_detector_node
  ${NVINFER_LIB}
  ${CUDA_CUDART_LIBRARY}
  ${OpenCV_LIBS}
)
ament_target_dependencies(trt_detector_node rclcpp sensor_msgs vision_msgs std_msgs)

install(TARGETS trt_detector_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
