cmake_minimum_required(VERSION 3.5)
project(xsens_mti_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# try to use ccache if found
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif(CCACHE_PROGRAM)

# force colored output on compile
option(FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)." ON)

if(${FORCE_COLORED_OUTPUT})
  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    add_compile_options(-fdiagnostics-color=always)
  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    add_compile_options(-fcolor-diagnostics)
  endif()
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/xspublic_targets.targets)

add_executable(
	xsens_mti_driver
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc
	${CMAKE_CURRENT_SOURCE_DIR}/src/xda_interface.cc
	${CMAKE_CURRENT_SOURCE_DIR}/src/xda_callback.cc
)

target_include_directories(xsens_mti_driver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic)
target_link_libraries(xsens_mti_driver xscontroller xscommon xstypes Threads::Threads)
  
ament_target_dependencies(xsens_mti_driver
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs)
  
install(TARGETS
  xsens_mti_driver
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
