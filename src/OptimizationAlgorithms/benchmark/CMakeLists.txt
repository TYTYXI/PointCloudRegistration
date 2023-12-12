set(sources
        OABenchmark.cpp
)

add_ons_test(OptimizationAlgorithms_benchmark
        SOURCES ${sources}
        DEPENDS OptimizationAlgorithms Eigen3::Eigen TBB::tbb PCRUtilities)

set(LIBRARY_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)
set(EXECUTABLE_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)