cmake_minimum_required(VERSION 3.8)
project(alicia_d_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/geomagic_cartesian_teleop.py
  scripts/geomagic_omni_state_adapter.py
  scripts/joint_trajectory_probe_measure.py
  scripts/servo_cartesian_probe_measure.py
  scripts/teleop_safety_filter.py
  scripts/trajectory_deadman_gate.py
  scripts/twist_preview_integrator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
