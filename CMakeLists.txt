cmake_minimum_required(VERSION 3.8)
project(rf_joy)

# rclcpp(Humble)은 C++17을 요구한다
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# rf_publisher_node는 시리얼 전용 읽기 스레드를 사용한다
find_package(Threads REQUIRED)

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

add_executable(rf_publisher_node src/rf_publisher.cpp)
add_executable(rf_to_joy_node src/rf_to_joy.cpp)

ament_target_dependencies(rf_publisher_node rclcpp std_msgs)
ament_target_dependencies(rf_to_joy_node rclcpp std_msgs sensor_msgs)

# libserial은 rf_publisher_node만 쓰므로 전역이 아니라 대상별로 붙인다
target_include_directories(rf_publisher_node PRIVATE include ${LIBSERIAL_INCLUDE_DIR})
target_link_libraries(rf_publisher_node ${LIBSERIAL_LIB} Threads::Threads)

install(TARGETS
  rf_publisher_node
  rf_to_joy_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  # package.xml의 ament_lint_auto / ament_lint_common test_depend를 실제로 실행한다
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # i-BUS 프레임 파싱은 하드웨어 없이 검증할 수 있으므로 단위 테스트로 묶어 둔다
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ibus_protocol test/test_ibus_protocol.cpp)
  target_include_directories(test_ibus_protocol PRIVATE include)
endif()

ament_package()
