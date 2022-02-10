cmake_minimum_required(VERSION 3.5)
project(k3lso_moteus)

if(NOT CMAKE_CXX_STANDARD)
  # set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CAN_LIBS pthread rt pcanbasic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include/)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(k3lso_msgs REQUIRED)

file(GLOB MOTEUS_PCAN_SRC src/moteus_pcan/*.cpp)

add_executable(k3lso_moteus_node src/k3lso_moteus_node.cpp ${MOTEUS_PCAN_SRC})
target_link_libraries(k3lso_moteus_node ${CAN_LIBS})
ament_target_dependencies(k3lso_moteus_node rclcpp std_msgs sensor_msgs k3lso_msgs)

install(TARGETS
k3lso_moteus_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
