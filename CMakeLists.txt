cmake_minimum_required(VERSION 3.16)
project(ZiYuSerialPort)

if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD 14) # 模板变量C++14后才支持

#CSerialPort
set(CSerialPortRootPath "/usr/local/")

include_directories(${CSerialPortRootPath}/include)
link_directories(${CSerialPortRootPath}/lib)
#link_directories(${CSerialPortRootPath}/lib)

include_directories(include/ZiYuSerialPort)

add_executable(ZiYuSerialPort
        main.cpp
        ZiYuSerialPort.hpp)

target_link_libraries( ${PROJECT_NAME} cserialport)