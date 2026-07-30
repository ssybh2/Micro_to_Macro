cmake_minimum_required(VERSION 3.8)
project(mujoco_micro)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

add_library(mujoco_micro_core
  src/kinematics.cpp
  src/control_core.cpp
)
target_include_directories(mujoco_micro_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(mujoco_micro_node src/mujoco_micro_node.cpp)
target_link_libraries(mujoco_micro_node mujoco_micro_core)
ament_target_dependencies(mujoco_micro_node
  rclcpp
  sensor_msgs
  std_msgs
  custom_msgs
)

add_executable(mujoco_micro_core_self_check tools/core_self_check.cpp)
target_link_libraries(mujoco_micro_core_self_check mujoco_micro_core)

install(TARGETS
  mujoco_micro_core
  mujoco_micro_node
  mujoco_micro_core_self_check
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/check_topics.sh
  scripts/check_message_api.sh
  scripts/mujoco_micro_debug_monitor.py
  scripts/generate_lqr_gains.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(mujoco_micro_core)
ament_package()
