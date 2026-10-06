cmake_minimum_required(VERSION 3.8)
project(so101_scan_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The whole stack without hardware: the real STS3215 driver on the fake servo bus, the
  # trajectory controller, the mirror bridge on the simulated ESP32, and a scan plan.
  ament_add_pytest_test(test_scan_stack test/test_scan_stack.py TIMEOUT 300)
endif()

ament_package()
