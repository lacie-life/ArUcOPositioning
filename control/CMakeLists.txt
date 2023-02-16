cmake_minimum_required(VERSION 2.8.12)

project(control)

if(NOT MSVC)
    add_definitions("-std=c++11 -Wall -Wextra -Werror")
else()
    add_definitions("-std=c++11 -WX -W2")
endif()

find_package(MAVSDK REQUIRED)

add_executable(control
    main.cpp
)

target_link_libraries(control
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk
    MAVSDK::mavsdk_offboard
)