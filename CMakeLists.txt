cmake_minimum_required(VERSION 3.5)

project(multilateration)

find_package(Ceres REQUIRED)

add_executable(multilateration multilateration.cpp)
target_link_libraries(multilateration Ceres::ceres)
