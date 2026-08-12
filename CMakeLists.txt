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

# libserial 찾기.
# libserial-dev가 설치해 주는 libserial.pc를 먼저 쓴다. 경로를 직접 적으면 배포판마다
# 다른 멀티아치 디렉터리(x86_64-linux-gnu, aarch64-linux-gnu 등)를 매번 놓치게 된다.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBSERIAL QUIET libserial)
endif()

if(LIBSERIAL_FOUND)
  # pkg_check_modules는 LIBSERIAL_INCLUDE_DIRS를 이미 채워 준다. 라이브러리는 이름만 담긴
  # LIBSERIAL_LIBRARIES 대신 절대 경로가 담긴 LIBSERIAL_LINK_LIBRARIES 쪽을 쓴다.
  set(LIBSERIAL_LIBRARIES ${LIBSERIAL_LINK_LIBRARIES})
else()
  # pkg-config가 없거나 .pc를 설치하지 않는 환경(직접 빌드한 libserial 등)을 위한 대비책.
  find_path(LIBSERIAL_INCLUDE_DIR NAMES libserial/SerialPort.h)
  find_library(LIBSERIAL_LIB NAMES serial)

  if(NOT LIBSERIAL_INCLUDE_DIR OR NOT LIBSERIAL_LIB)
    message(FATAL_ERROR
      "libserial not found. Install it with: sudo apt install libserial-dev")
  endif()

  set(LIBSERIAL_INCLUDE_DIRS ${LIBSERIAL_INCLUDE_DIR})
  set(LIBSERIAL_LIBRARIES ${LIBSERIAL_LIB})
endif()

add_executable(rf_publisher_node src/rf_publisher.cpp)
add_executable(rf_to_joy_node src/rf_to_joy.cpp)

ament_target_dependencies(rf_publisher_node rclcpp std_msgs)
ament_target_dependencies(rf_to_joy_node rclcpp std_msgs sensor_msgs)

# libserial은 rf_publisher_node만 쓰므로 전역이 아니라 대상별로 붙인다
target_include_directories(rf_publisher_node PRIVATE include ${LIBSERIAL_INCLUDE_DIRS})
target_link_libraries(rf_publisher_node ${LIBSERIAL_LIBRARIES} Threads::Threads)

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

# udev 규칙은 설치 단계에서 시스템에 복사하지 않는다. /etc에 쓰려면 root 권한이 필요하고
# colcon build는 사용자 권한으로 도는 것이 정상이기 때문이다. 대신 share/에 같이 깔아 두어
# 설치된 워크스페이스만 있어도 규칙 파일을 찾아 복사할 수 있게 한다.
#   sudo cp $(ros2 pkg prefix rf_joy)/share/rf_joy/udev/99-ibus-rf.rules /etc/udev/rules.d/
# USE_SOURCE_PERMISSIONS가 없으면 install_udev_rules.sh의 실행 권한이 떨어져 나간다.
install(DIRECTORY udev/
  DESTINATION share/${PROJECT_NAME}/udev
  USE_SOURCE_PERMISSIONS
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
