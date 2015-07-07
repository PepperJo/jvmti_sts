cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_COLOR_MAKEFILE ON)

project(jvmti_sts)

string(TOUPPER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE)

IF(CMAKE_BUILD_TYPE MATCHES DEBUG)
    MESSAGE(STATUS "DEBUG mode")
    SET(CMAKE_VERBOSE_MAKEFILE ON)
    SET(CMAKE_BUILD_TYPE DEBUG)
    SET(Boost_LIBRARY libboost_system-mt.so)
ELSEIF (CMAKE_BUILD_TYPE MATCHES RELEASE)
    MESSAGE(STATUS "RELEASE mode")
    SET(CMAKE_BUILD_TYPE RELEASE)
    SET(Boost_LIBRARY libboost_system-mt.so)
ELSE()
    MESSAGE(STATUS "No build type specified, setting build type to RELEASE mode")
    SET(CMAKE_BUILD_TYPE RELEASE)
ENDIF()

##
# Set java and boost directories in this file
include("${PROJECT_SOURCE_DIR}/config.local" OPTIONAL)

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
#execute_process(COMMAND "echo ${Boost_LIBRARY_DIRS} | sed 's/lib64\\/lib64/lib64/g'" OUTPUT_VARIABLE Boost_New_LIBRARY_DIRS)
link_directories(${Boost_LIBRARY_DIRS})

IF(${CMAKE_BUILD_TYPE} MATCHES DEBUG)
    get_cmake_property(_variableNames VARIABLES)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
ENDIF()

##
# Subdirectories
add_subdirectory(agent)
add_subdirectory(control)

