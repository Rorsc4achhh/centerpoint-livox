cmake_minimum_required(VERSION 3.5)
project(pcl_reg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Add compile options, recommended for ROS2
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_executable(reg_pcl src/reg_pcl.cpp)
ament_target_dependencies(reg_pcl rclcpp sensor_msgs pcl_conversions)
target_link_libraries(reg_pcl ${PCL_LIBRARIES})

install(TARGETS
  reg_pcl
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

