cmake_minimum_required(VERSION 2.8.9)
project(cubic_spline)

#Bring the headers, such as Student.h into the project
include_directories(include)


file(GLOB SOURCES 
                include/cubic.h)

# add_executable(object_test src/object_test.cpp)
# add_executable(cubic_spline src/cubic_spline.cpp)
add_executable(cubic src/cubic.cpp)

