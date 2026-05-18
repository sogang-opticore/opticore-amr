cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# 의존성
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---------------------------------------------------------------------------
# C++ 헤더 (HU의 astar_planner.hpp 등 — 현재는 placeholder)
# ---------------------------------------------------------------------------
include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include/
)

# ---------------------------------------------------------------------------
# Python 패키지 install — SW의 dwa_node, heuristics 등
# ---------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/dwa_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# launch / config 설치
# ---------------------------------------------------------------------------
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# 테스트
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)    # docstring D213 vs D212 충돌 회피 (Week 1 이슈 #7)
  ament_lint_auto_find_test_dependencies()

  # Python 단위 테스트
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_heuristics
    test/test_heuristics.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_dwa
    test/test_dwa.py
    TIMEOUT 60
  )
endif()

ament_package()
