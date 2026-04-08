cmake_minimum_required(VERSION 3.16)

project(servo_hub_qt VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Widgets SerialPort)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets SerialPort)

add_executable(servo_hub_qt
    src/main.cpp
    src/app/MainWindow.h
    src/app/MainWindow.cpp
    src/app/TorqueBarWidget.h
    src/app/TorqueBarWidget.cpp
    src/communication/AbstractBusWorker.h
    src/communication/CanBusWorker.h
    src/communication/CanBusWorker.cpp
    src/communication/CommunicationManager.h
    src/communication/CommunicationManager.cpp
    src/communication/SerialCanFrameCodec.h
    src/communication/SerialCanFrameCodec.cpp
    src/models/MotorTableModel.h
    src/models/MotorTableModel.cpp
    src/models/Types.h
    src/models/Types.cpp
    src/protocols/CanOpenProtocol.h
    src/protocols/CanOpenProtocol.cpp
    src/services/MotorService.h
    src/services/MotorService.cpp
)

target_include_directories(servo_hub_qt PRIVATE src)

target_link_libraries(servo_hub_qt PRIVATE
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::SerialPort
)
