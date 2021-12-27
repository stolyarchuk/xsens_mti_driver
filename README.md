# Xsens MTi driver for ROS

## Documentation

You can find the full documentation in "<your MT SDK directory>/doc/xsensdeviceapi/doc/html/index.html" under "ROS MTi driver" section.

## Prerequisites

- ROS2 foxy
- C/C++ Compiler: GCC 9
- C++17

## Building options

- ```XSENS_USE_XDA``` - use prebuilt external Xsens SDK instead of xspublic (disabled by default)
- ```XSENS_INSTALL_PATH``` - external Xsens SDK install directory (default is `/usr/local/xsens`)
