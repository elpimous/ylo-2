cmake_minimum_required(VERSION 3.0.2)
project(wolf_ylo2_interface)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  realtime_tools
  wolf_hardware_interface
)
      
#add_subdirectory(ylo2_hal)

catkin_package(
 CATKIN_DEPENDS
   controller_manager
   realtime_tools
   wolf_hardware_interface
)

##########
# Build ##
##########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ylo2_ros_node
    src/ylo2_robot_hw.cpp
    src/ylo2_ros_control.cpp
    src/ylo2_ros_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(ylo2_ros_node
  ${catkin_LIBRARIES}
  #ylo2hal
)

set_property(TARGET ylo2_ros_node PROPERTY CXX_STANDARD 11)  # At least std c++11

#############
## Install ##
#############

install(
  TARGETS ylo2_ros_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

