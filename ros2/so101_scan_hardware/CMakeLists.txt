cmake_minimum_required(VERSION 3.8)
project(so101_scan_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/sts_protocol.cpp
  src/serial_port.cpp
  src/sts_bus.cpp
  src/feetech_sts_system.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface so101_scan_hardware.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# The tools: bus scan, calibration, fake bus
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/sts_scan
  scripts/sts_calibrate
  scripts/sts_fake_bus
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_add_gtest(test_sts_protocol test/test_sts_protocol.cpp)
  target_link_libraries(test_sts_protocol ${PROJECT_NAME})

  # The C++ driver against the Python fake bus
  ament_add_gtest(test_driver_fake_bus test/test_driver_fake_bus.cpp
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60)
  target_link_libraries(test_driver_fake_bus ${PROJECT_NAME})
  ament_target_dependencies(test_driver_fake_bus lifecycle_msgs)
  target_compile_definitions(test_driver_fake_bus PRIVATE PYTHON_EXECUTABLE="${Python3_EXECUTABLE}")

  ament_add_pytest_test(test_sts_tools test/test_sts_tools.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
