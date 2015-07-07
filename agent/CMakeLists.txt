cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(jvmti_sts)

string(TOUPPER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE)

IF(CMAKE_BUILD_TYPE MATCHES DEBUG)
    MESSAGE(STATUS "DEBUG mode")
    SET(CMAKE_VERBOSE_MAKEFILE ON)
    SET(CMAKE_BUILD_TYPE Debug)
ELSEIF (CMAKE_BUILD_TYPE MATCHES RELEASE)
    MESSAGE(STATUS "RELEASE mode")
    SET(CMAKE_BUILD_TYPE Release)
ELSE()
    MESSAGE(STATUS "No build type specified, setting build type to RELEASE mode")
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

##
# Set java and boost directories in this file
include("${PROJECT_SOURCE_DIR}/../config.local" OPTIONAL)

add_compile_options(-std=${CppSTD})
add_compile_options(-Wall -Werror)
add_compile_options(-D_GLIBCXX_USE_NANOSLEEP)

#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Werror -std=${CppSTD} -D_GLIBCXX_USE_NANOSLEEP")

IF(!JAVA_DIR)
    SET(JAVA_DIR ${/usr/lib/jvm/java-1.7.0-openjdk-amd64})
ENDIF()
MESSAGE(STATUS "JAVA_DIR is set to ${JAVA_DIR}")

include_directories(${JAVA_DIR}/include)

find_package(Boost COMPONENTS system REQUIRED)
include_directories(${Boost_INCLUDE_DIR} "../control")

#set(LIBRARY_OUTPUT_PATH ${dir})

add_library(jvmti_sts SHARED jvmti_sts.cpp)
target_link_libraries(jvmti_sts ${Boost_LIBRARIES})
# set_property(TARGET jvmti_sts PROPERTY CXX_STANDARD 11)
# set_property(TARGET jvmti_sts PROPERTY CXX_STANDARD_REQUIRED ON)
