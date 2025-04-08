cmake_minimum_required(VERSION 3.8)
project(p6_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  rclcpp_lifecycle
  geometry_msgs
  kobuki_ros_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  vision_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/p6_package/CycleNode.cpp
  src/p6_package/TFPublisherNode.cpp
  src/p6_package/PIDController.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Vincula los ejecutables con la biblioteca creada previamente
add_executable(cyclenode_main src/cyclenode_main.cpp)
target_link_libraries(cyclenode_main ${PROJECT_NAME})
ament_target_dependencies(cyclenode_main ${dependencies})

add_executable(tf_publisher_main src/tf_publisher_main.cpp)
target_link_libraries(tf_publisher_main ${PROJECT_NAME})
ament_target_dependencies(tf_publisher_main ${dependencies})

install(TARGETS
  ${PROJECT_NAME}
  cyclenode_main
  tf_publisher_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
