cmake_minimum_required(VERSION 3.0.0)
project(INDOORLOOSECOMB VERSION 0.1.0)

# 若不是debug进行o3优化
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    message("o3 ${CMAKE_BUILD_TYPE}")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
else()
    message("${CMAKE_BUILD_TYPE}")
endif()
# c++11标准
set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

include_directories(
    "D:/software/programer/LIB/eigen-3.3.9/"
)
AUX_SOURCE_DIRECTORY(. DIR_SRCS)
message(${DIR_SRCS})
# add_executable(testmech testmech.cpp ${DIR_SRCS})
add_executable(testekf testekf.cpp ${DIR_SRCS})