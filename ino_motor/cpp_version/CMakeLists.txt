cmake_minimum_required(VERSION 3.0.2)
project(inovance_servo_driver)

## 查找 catkin 和依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## 查找 Eigen3
find_package(Eigen3 REQUIRED)

###################################
## catkin 配置 ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES inovance_servo_driver
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## 指定头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## 编译 USB-CAN 库
add_library(usb_can_lib
  hw_can_usb.cpp
)
target_link_libraries(usb_can_lib
  ${catkin_LIBRARIES}
)

## 编译 Inovance Servo 库
add_library(inovance_servo_lib
  inovance_servo.cpp
)
target_link_libraries(inovance_servo_lib
  usb_can_lib
  ${catkin_LIBRARIES}
)

## 编译测试程序
add_executable(servo_test
  servo_test.cpp
)
target_link_libraries(servo_test
  inovance_servo_lib
  usb_can_lib
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## 安装可执行文件
install(TARGETS servo_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装库
install(TARGETS usb_can_lib inovance_servo_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## 安装头文件
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
