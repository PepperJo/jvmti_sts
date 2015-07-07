cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)

include_directories(${JAVA_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/control)

add_library(jvmti_sts SHARED jvmti_sts.cpp) 
target_link_libraries(jvmti_sts ${Boost_LIBRARY})
