cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(jvmti_sts)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Werror -std=c++11")

include_directories(/usr/lib/jvm/java-1.7.0-openjdk-amd64/include)

find_package( Boost 1.55 COMPONENTS system REQUIRED)
include_directories(${Boost_INCLUDE_DIR} "../control")

add_library(jvmti_sts SHARED jvmti_sts.cpp)
target_link_libraries(jvmti_sts ${Boost_LIBRARIES})
# set_property(TARGET jvmti_sts PROPERTY CXX_STANDARD 11)
# set_property(TARGET jvmti_sts PROPERTY CXX_STANDARD_REQUIRED ON)
