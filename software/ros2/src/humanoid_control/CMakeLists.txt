cmake_minimum_required(VERSION 3.8)
project(humanoid_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(humanoid_infrastructure REQUIRED)

include_directories(include)

add_executable(joint_trajectory_bridge_node src/joint_trajectory_bridge_node.cpp)
ament_target_dependencies(joint_trajectory_bridge_node
  rclcpp
  trajectory_msgs
  sensor_msgs
  geometry_msgs
  humanoid_interfaces
  humanoid_infrastructure
)

install(TARGETS joint_trajectory_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mico_gait.py
  scripts/mico_playback.py
  scripts/mico_mujoco_bridge.py
  scripts/mico_rl_node.py
  scripts/train_mico_ppo.py
  scripts/benchmark_mico_sim.py
  scripts/model_registry.py
  scripts/eval_mico_benchmark.py
  scripts/mico_visualizer.py
  scripts/auto_train_mico_evolution.py
  DESTINATION lib/${PROJECT_NAME}
)
# imported by mico_mujoco_bridge.py and mico_rl_node.py from the same folder
install(FILES
  scripts/mico_mujoco.py
  scripts/mico_rl_env.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # FK and stance-foot odometry of mico_playback.py, numpy only
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mico_playback test/test_mico_playback.py TIMEOUT 120)
  # MuJoCo bridge core; skips itself where MuJoCo is not installed (the Ubuntu CI)
  ament_add_pytest_test(test_mico_mujoco test/test_mico_mujoco.py TIMEOUT 300)
  # Reinforcement learning env & ONNX policy
  ament_add_pytest_test(test_mico_rl test/test_mico_rl.py TIMEOUT 120)
endif()

ament_export_include_directories(include)
ament_package()
