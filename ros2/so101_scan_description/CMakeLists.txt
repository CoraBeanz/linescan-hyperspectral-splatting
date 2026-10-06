cmake_minimum_required(VERSION 3.8)
project(so101_scan_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The SO-101 arm meshes are not committed (17 MB). They live in cad/vendor/so101/assets,
# where cad/vendor/so101/fetch_assets.py downloads them; fetch them now if that hasn't run.
get_filename_component(SO101_VENDOR ${CMAKE_CURRENT_SOURCE_DIR}/../../cad/vendor/so101 ABSOLUTE)
if(NOT EXISTS ${SO101_VENDOR}/assets/base_so101_v2.stl)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  message(STATUS "Downloading the SO-101 meshes into ${SO101_VENDOR}/assets")
  execute_process(COMMAND ${Python3_EXECUTABLE} ${SO101_VENDOR}/fetch_assets.py
    RESULT_VARIABLE fetch_result)
  if(NOT fetch_result EQUAL 0)
    message(WARNING "Could not download the SO-101 meshes, so RViz will show the arm without "
      "them. Run python3 cad/vendor/so101/fetch_assets.py and build again.")
  endif()
endif()
if(EXISTS ${SO101_VENDOR}/assets)
  install(DIRECTORY ${SO101_VENDOR}/assets/ DESTINATION share/${PROJECT_NAME}/meshes/so101
    FILES_MATCHING PATTERN "*.stl")
endif()

install(DIRECTORY launch meshes rviz urdf DESTINATION share/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description test/test_description.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120)
endif()

ament_package()
