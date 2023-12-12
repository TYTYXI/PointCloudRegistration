set(sources
        registration_global.h
        registrationObject.h registrationObject.cpp
        registration_visualization.h
        registration_visualization.cpp
        #        tlboRegistraion.h tlboRegistraion.cpp
        helper_cuda.h helper_string.h
        dist.h
)

add_ons_lib(Registration
        PUBLIC_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}
        INCLUDES ../OptimizationAlgorithms pcl/gpu/containers/include ../utilities
        SOURCES ${sources}
        DEPENDS Qt::Core ${PCL_LIBRARIES} ${VTK_LIBRARIES}
        Eigen3::Eigen TBB::tbb
        LIB_DEPENDS OptimizationAlgorithms PCRUtilities dataset
        COMPILE_DEFINITIONS REGISTRATION_LIB
)

add_subdirectory(problems)
add_subdirectory(tests)
add_subdirectory(benchmark)
add_subdirectory(filter)

set(LIBRARY_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)
set(EXECUTABLE_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)