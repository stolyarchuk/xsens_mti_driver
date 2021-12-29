cmake_minimum_required(VERSION 3.5)
project(xsens_mti_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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

add_executable(
	xsens_mti_driver
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc
	${CMAKE_CURRENT_SOURCE_DIR}/src/xda_interface.cc
	${CMAKE_CURRENT_SOURCE_DIR}/src/xda_callback.cc
)

# xsens dependency variants
option(XSENS_USE_XDA "Use Xsens SDK." OFF)

if(NOT XSENS_USE_XDA)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/xspublic_targets.cmake)
  set(XDA_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic)
  set(XDA_LIBS xscontroller xscommon xstypes)
else()
  if(NOT XSENS_INSTALL_PREFIX)
    set(XSENS_INSTALL_PREFIX /usr/local/xsens)
  endif()
  find_library(XSENSDEVICEAPI_LIB libxsensdeviceapi.so ${XSENS_INSTALL_PREFIX}/lib)
  find_library(XSTYPES_LIB libxstypes.so ${XSENS_INSTALL_PREFIX}/lib)
  target_compile_definitions(xsens_mti_driver PRIVATE XSENS_USE_XDA)
  set(XDA_INCLUDES ${XSENS_INSTALL_PREFIX}/include)
  set(XDA_LIBS ${XSENSDEVICEAPI_LIB} ${XSTYPES_LIB})
endif()

target_include_directories(xsens_mti_driver PUBLIC ${XDA_INCLUDES})
target_link_libraries(xsens_mti_driver Threads::Threads ${XDA_LIBS})
  
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
