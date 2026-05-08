cmake_minimum_required(VERSION 3.8)
project(remora_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


find_package(PkgConfig REQUIRED)
pkg_check_modules(GeographicLib REQUIRED IMPORTED_TARGET geographiclib)

add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs tf2_ros tf2 ${Eigen_LIBRARIES})

add_executable(GoTo src/GoTo.cpp)
ament_target_dependencies(GoTo rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

add_executable(SwarmAggregation src/SwarmAggregation.cpp)
ament_target_dependencies(SwarmAggregation rclcpp geometry_msgs sensor_msgs)

add_executable(state_estimator src/state_estimator.cpp)
ament_target_dependencies(state_estimator rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs)
target_link_libraries(state_estimator PkgConfig::GeographicLib)




include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

install(DIRECTORY include
  DESTINATION include
)


install(TARGETS
  simple_controller
  GoTo
  state_estimator
  SwarmAggregation
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)




ament_package()
