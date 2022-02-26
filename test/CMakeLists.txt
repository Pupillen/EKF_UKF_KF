cmake_minimum_required(VERSION 3.0.2)
project(test)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
find_package(Eigen3 REQUIRED)
include_directories(
  ${EIGEN3_INCLUDE_DIR} 
  include
../include
  )
add_library(filter ../src/ekf.cpp ../include/ekf.hpp)
add_executable(main src/main.cpp)
add_executable(filtertest src/main4.cpp)
target_link_libraries(filtertest LINK_PUBLIC filter)
#set(EXECUTABLE_OUTPUT_PATH ./debug)
