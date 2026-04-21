cmake_minimum_required(VERSION 3.16)
project(dual_arm_control VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets Core SerialPort Network)

set(HUMANOID_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../HumanoidArms/HumanoidArms_SDK")
set(HUMANOID_SDK_INCLUDE_DIR "${HUMANOID_SDK_DIR}/include")
set(HUMANOID_SDK_LIB_DIR "${HUMANOID_SDK_DIR}/usrlib")
set(HUMANOID_SDK_LIB "${HUMANOID_SDK_LIB_DIR}/libHumanoidArms.so")
set(HUMANOID_SDK_CONTROLCAN_LIB "${HUMANOID_SDK_LIB_DIR}/libcontrolcan.so")
set(LOCAL_EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen3_pkg")
set(LOCAL_BOOST_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")

if(NOT EXISTS "${HUMANOID_SDK_LIB}")
    message(FATAL_ERROR "HumanoidArms SDK not found: ${HUMANOID_SDK_LIB}")
endif()

if(EXISTS "${LOCAL_EIGEN3_INCLUDE_DIR}/Eigen/Dense")
    set(EIGEN3_INCLUDE_HINT "${LOCAL_EIGEN3_INCLUDE_DIR}")
else()
    set(EIGEN3_INCLUDE_HINT "/usr/include/eigen3")
endif()

find_library(NCURSES_LIB
    NAMES libncurses.so.6 ncurses ncursesw
    PATHS /lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu
    NO_DEFAULT_PATH
)

if(NOT NCURSES_LIB)
    message(FATAL_ERROR "ncurses runtime library not found")
endif()

set(MODEL_SOURCES
    models/common_types.h
)

set(COMMUNICATION_SOURCES
    communication/ican_adapter.h
    communication/serial_can_adapter.h
    communication/serial_can_adapter.cpp
    communication/vci_can_adapter.h
    communication/vci_can_adapter.cpp
    communication/controlcan.h
    communication/can_bus_worker.h
    communication/can_bus_worker.cpp
    communication/communication_manager.h
    communication/communication_manager.cpp
)

set(PROTOCOL_SOURCES
    protocols/canopen_motor_protocol.h
    protocols/canopen_motor_protocol.cpp
    protocols/humanoid_arms_sdk_adapter.h
    protocols/humanoid_arms_sdk_adapter.cpp
    protocols/ze300_protocol.h
    protocols/ze300_protocol.cpp
    protocols/cybergear_protocol.h
    protocols/cybergear_protocol.cpp
)

set(SERVICE_SOURCES
    services/motor_service.h
    services/motor_service.cpp
    services/arm_service.h
    services/arm_service.cpp
    services/ze300_service.h
    services/ze300_service.cpp
    services/cybergear_service.h
    services/cybergear_service.cpp
    services/gripper_worker.h
    services/gripper_worker.cpp
    services/gripper_service.h
    services/gripper_service.cpp
)

set(UI_SOURCES
    ui/main_window.h
    ui/main_window.cpp
    ui/connection_panel.h
    ui/connection_panel.cpp
    ui/motor_control_panel.h
    ui/motor_control_panel.cpp
    ui/status_monitor_panel.h
    ui/status_monitor_panel.cpp
    ui/torque_gauge_widget.h
    ui/torque_gauge_widget.cpp
    ui/log_widget.h
    ui/log_widget.cpp
    ui/humanoid_arms_panel.h
    ui/humanoid_arms_panel.cpp
    ui/ze300_control_panel.h
    ui/ze300_control_panel.cpp
    ui/cybergear_control_panel.h
    ui/cybergear_control_panel.cpp
    ui/gripper_control_panel.h
    ui/gripper_control_panel.cpp
)

add_executable(dual_arm_control
    main.cpp
    ${MODEL_SOURCES}
    ${COMMUNICATION_SOURCES}
    ${PROTOCOL_SOURCES}
    ${SERVICE_SOURCES}
    ${UI_SOURCES}
)

target_include_directories(dual_arm_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${HUMANOID_SDK_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_HINT}
    ${LOCAL_BOOST_INCLUDE_DIR}
)

set_target_properties(dual_arm_control PROPERTIES
    BUILD_RPATH "${HUMANOID_SDK_LIB_DIR}"
    INSTALL_RPATH "${HUMANOID_SDK_LIB_DIR}"
)

target_link_options(dual_arm_control PRIVATE "-Wl,--no-as-needed")

# libcontrolcan.so is now loaded dynamically via dlopen in VciCanAdapter
# to prevent resource contention and startup crashes.

target_link_libraries(dual_arm_control PRIVATE
    Qt6::Widgets
    Qt6::Core
    Qt6::SerialPort
    Qt6::Network
    ${HUMANOID_SDK_LIB}
    ${HUMANOID_SDK_CONTROLCAN_LIB}
    ${NCURSES_LIB}
    ${CMAKE_DL_LIBS}
)
