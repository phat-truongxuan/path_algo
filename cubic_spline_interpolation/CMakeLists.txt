cmake_minimum_required(VERSION 2.8.9)
project(cubic_spline)

include_directories(include)


file(GLOB SOURCES 
                include/cubic.h)


add_executable(cubic src/cubic.cpp)

