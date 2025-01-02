cmake_minimum_required(VERSION 3.8)
project(rf_joy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# libserial 찾기
find_path(LIBSERIAL_INCLUDE_DIR
  NAMES libserial/SerialPort.h
  PATHS /usr/include /usr/local/include
)

find_library(LIBSERIAL_LIB
  NAMES serial
  PATHS /usr/lib /usr/local/lib
)


if(NOT LIBSERIAL_INCLUDE_DIR OR NOT LIBSERIAL_LIB)
  message(FATAL_ERROR "libserial not found. Please install libserial-dev.")
endif()


include_directories(${LIBSERIAL_INCLUDE_DIR})

add_executable(rf_publisher_node src/rf_publisher.cpp)
add_executable(rf_to_joy_node src/rf_to_joy.cpp)

ament_target_dependencies(rf_publisher_node rclcpp std_msgs)
ament_target_dependencies(rf_to_joy_node rclcpp std_msgs sensor_msgs)

target_link_libraries(rf_publisher_node ${LIBSERIAL_LIB})
target_link_libraries(rf_to_joy_node ${LIBSERIAL_LIB})

install(TARGETS
  rf_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  rf_to_joy_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
