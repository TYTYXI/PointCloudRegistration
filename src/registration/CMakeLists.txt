set(sources
        registration_global.h
        registrationObject.h registrationObject.cpp
        registraion_visualization.h
        registraion_visualization.cpp
#        tlboRegistraion.h tlboRegistraion.cpp
        L2_SimpleProblem.cpp L2_SimpleProblem.h
        dist.h
        )

add_ons_plugin(registration
        INCLUDES ../OptimizationAlgorithms
        SOURCES ${sources}
        PUBLIC_DEPENDS Qt::Core ${PCL_LIBRARIES} Eigen3::Eigen TBB::tbb OptimizationAlgorithms)

add_ons_test(registration
        INCLUDES ../OptimizationAlgorithms
        SOURCES registration_test.cpp ${sources} WOARegsitrationTest.cpp
        DEPENDS Qt::Core registration dataset ${PCL_LIBRARIES} OptimizationAlgorithms Eigen3::Eigen TBB::tbb)

add_ons_test(registration_benchmark
        INCLUDES ../OptimizationAlgorithms
        SOURCES registration_benchmark.cpp ${sources}
        DEPENDS Qt::Core registration dataset ${PCL_LIBRARIES} OptimizationAlgorithms Eigen3::Eigen TBB::tbb)

#target_compile_definitions(algorithm PRIVATE ALGORITHM_LIB)

set(LIBRARY_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)
set(EXECUTABLE_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)