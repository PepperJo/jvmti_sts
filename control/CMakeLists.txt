cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(${Boost_INCLUDE_DIR})

set(EXECUTABLE_OUTPUT_PATH ${dir})

IF(CMAKE_BUILD_TYPE MATCHES DEBUG)
get_cmake_property(_variableNames VARIABLES)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()
ENDIF()

add_executable(sts_control sts_control.cpp)
target_link_libraries(sts_control pthread)
target_link_libraries(sts_control ${Boost_LIBRARY})

# set_property(TARGET sts_control PROPERTY CXX_STANDARD 11)
# set_property(TARGET sts_control PROPERTY CXX_STANDARD_REQUIRED ON)
