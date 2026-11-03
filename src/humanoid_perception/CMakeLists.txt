cmake_minimum_required(VERSION 3.8)
project(humanoid_perception)

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
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(humanoid_infrastructure REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 1. Perception Node (Camera stream subscriber + Sync)
add_executable(camera_subscriber_node src/camera_subscriber_node.cpp)
ament_target_dependencies(camera_subscriber_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_filters
  humanoid_interfaces
  humanoid_infrastructure
)
target_link_libraries(camera_subscriber_node ${OpenCV_LIBRARIES})

# 2. ONNX / AI Inference Pipeline Node
add_executable(onnx_inference_node src/onnx_inference_node.cpp)
ament_target_dependencies(onnx_inference_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_filters
  humanoid_interfaces
  humanoid_infrastructure
)
target_link_libraries(onnx_inference_node ${OpenCV_LIBRARIES})

install(TARGETS
  camera_subscriber_node
  onnx_inference_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  humanoid_interfaces
  humanoid_infrastructure
)
ament_package()
