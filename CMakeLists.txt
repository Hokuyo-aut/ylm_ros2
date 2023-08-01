cmake_minimum_required(VERSION 3.5)
project(ylm_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED)

set(YLM_INCLUDE_DIRS
    "include;")

include_directories(include ${YLM_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(ylm_client STATIC src/lumotive_client_interface.cpp src/ylm_client.cpp)
target_link_libraries(ylm_client ${Boost_LIBRARIES})

add_executable(lumotive_driver src/lumotive_driver.cpp src/lumotive_cloud_creation.cpp)
ament_target_dependencies(lumotive_driver rclcpp sensor_msgs)
target_link_libraries(lumotive_driver ylm_client ${rclcpp_LIBRARIES})

install(TARGETS
  lumotive_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
