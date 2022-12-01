cmake_minimum_required(VERSION 3.16)

project(PointCloudRegistration)

set(CMAKE_CXX_STANDARD 20)

#设置CMAKE模块位置
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

include(SpatialRegistrationAPI)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(TBB REQUIRED)
find_package(OPENMP REQUIRED)


set(BIN_OUTPUT_PATH ${PROJECT_BINARY_DIR}/OtronicNavigationSystem)

add_subdirectory(benchmark/googletest)
add_subdirectory(benchmark)

add_subdirectory(src)

add_subdirectory(tests)