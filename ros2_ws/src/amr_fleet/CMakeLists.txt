cmake_minimum_required(VERSION 3.8)
project(amr_fleet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # F-3: GATE-F3-LOGIC — fleet_priority 순수코어 단위테스트 (sim 불필요·결정론)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fleet_priority test/test_fleet_priority.py
    TIMEOUT 120
  )
endif()

# Python 노드 설치 (amr_slam 패턴). ament_cmake → 복사설치이므로 .py 수정 후 colcon build 필수.
install(PROGRAMS
  src/deadlock_manager.py
  src/priority_publisher.py
  src/f3_priority_node.py
  src/fleet_priority.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
