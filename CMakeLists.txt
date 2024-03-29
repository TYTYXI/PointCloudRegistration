cmake_minimum_required(VERSION 3.16)

project(PointCloudRegistration)

set(CMAKE_CXX_STANDARD 14)
#设置CMAKE模块位置
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

include(SpatialRegistrationAPI)

if (CMAKE_CXX_STANDARD EQUAL 17)
    add_compile_definitions(PCR_CXX17)
elseif (CMAKE_CXX_STANDARD EQUAL 20)
    add_compile_definitions(PCR_CXX20)
endif ()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
find_package(TBB REQUIRED)
find_package(Qt6 COMPONENTS
        Core
        REQUIRED)


find_package(OpenMP REQUIRED)

enable_language(CUDA)

set(BIN_OUTPUT_PATH ${PROJECT_BINARY_DIR}/OtronicNavigationSystem)

add_subdirectory(benchmark/googletest)
add_subdirectory(benchmark)

add_subdirectory(dataset)

add_subdirectory(src)

add_subdirectory(tests)
