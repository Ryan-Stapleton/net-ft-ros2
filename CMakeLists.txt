cmake_minimum_required(VERSION 3.8)
project(netft_rdt_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread program_options)
find_package(rosidl_default_generators REQUIRED)

set(SRV_FILES
  "srv/StringCmd.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  DEPENDENCIES std_msgs
)

set(SRC_FILES
  src/netft_rdt_driver.cpp
  src/netft_rdt_bias.cpp
  src/ft_listener.cpp
)

add_library(netft_rdt_driver_lib ${SRC_FILES})
target_include_directories(netft_rdt_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(netft_rdt_driver_lib
  rclcpp
  diagnostic_updater
  diagnostic_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(netft_node src/netft_node.cpp)
target_link_libraries(netft_node netft_rdt_driver_lib Boost::program_options)
ament_target_dependencies(netft_node
  rclcpp
  diagnostic_updater
  diagnostic_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  Boost
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(netft_rdt_driver_lib ${cpp_typesupport_target} Boost::thread)
target_link_libraries(netft_node ${cpp_typesupport_target})

install(
  TARGETS netft_rdt_driver_lib netft_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

ament_export_include_directories(include)
ament_export_libraries(netft_rdt_driver_lib)
ament_export_dependencies(
  rclcpp
  diagnostic_updater
  diagnostic_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)

ament_package()
