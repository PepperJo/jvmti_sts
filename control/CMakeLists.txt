cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(sts_control)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Werror")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package( Boost 1.55 COMPONENTS system REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_executable(sts_control sts_control.cpp)
target_link_libraries(sts_control ${Boost_LIBRARIES} pthread)
set_property(TARGET sts_control PROPERTY CXX_STANDARD 11)
set_property(TARGET sts_control PROPERTY CXX_STANDARD_REQUIRED ON)
