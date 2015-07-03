cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(sts_control)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Werror -std=c++11")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package( Boost COMPONENTS system REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

set(EXECUTABLE_OUTPUT_PATH ${dir})

add_executable(sts_control sts_control.cpp)
target_link_libraries(sts_control ${Boost_LIBRARIES} pthread)
# set_property(TARGET sts_control PROPERTY CXX_STANDARD 11)
# set_property(TARGET sts_control PROPERTY CXX_STANDARD_REQUIRED ON)
