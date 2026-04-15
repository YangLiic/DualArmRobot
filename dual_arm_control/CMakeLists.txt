cmake_minimum_required(VERSION 3.16)
project(dual_arm_control VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets Core SerialPort)

set(MODEL_SOURCES
    models/common_types.h
)

set(COMMUNICATION_SOURCES
    communication/frame_io.h
    communication/frame_io.cpp
    communication/can_bus_worker.h
    communication/can_bus_worker.cpp
    communication/communication_manager.h
    communication/communication_manager.cpp
)

set(PROTOCOL_SOURCES
    protocols/canopen_motor_protocol.h
    protocols/canopen_motor_protocol.cpp
)

set(SERVICE_SOURCES
    services/motor_service.h
    services/motor_service.cpp
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
)

add_executable(dual_arm_control
    main.cpp
    ${MODEL_SOURCES}
    ${COMMUNICATION_SOURCES}
    ${PROTOCOL_SOURCES}
    ${SERVICE_SOURCES}
    ${UI_SOURCES}
)

target_include_directories(dual_arm_control PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(dual_arm_control PRIVATE
    Qt6::Widgets
    Qt6::Core
    Qt6::SerialPort
)
