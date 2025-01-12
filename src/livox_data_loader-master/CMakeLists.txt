cmake_minimum_required(VERSION 3.5)
project(data_loader)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find_package(Boost REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)
find_package(ws_msgs REQUIRED)

add_library(data_loader SHARED
src/data_loader.cpp)

ament_target_dependencies(data_loader
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  OpenCV
  yaml-cpp
  ws_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${ws_msgs_INCLUDE_DIRS})

  add_executable(data_loader_node src/main.cpp)
  target_link_libraries(data_loader_node
    data_loader
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    yaml-cpp
  )
  ament_target_dependencies(data_loader_node
  "rclcpp"
)

install(TARGETS
    data_loader
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)


install(TARGETS
    data_loader_node
    DESTINATION lib/${PROJECT_NAME}
)


# Install launch files.
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

SET(CMAKE_BUILD_TYPE Debug) 

ament_package()
