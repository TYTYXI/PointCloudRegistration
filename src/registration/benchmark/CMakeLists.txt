set(sources
        registration_benchmark.cpp
        BM_IA_FPCS.h
        BM_IA_KFPCS.h
        BM_IA_RANSAC.h
        BM_ICP_P2N.h
        BM_ICP_P2P.h
        BM_MCTLBO.h
)

add_ons_test(Registration_Benchmark
        SOURCES ${sources}
        DEPENDS Qt::Core Registration dataset ${PCL_LIBRARIES} ${VTK_LIBRARIES} Eigen3::Eigen TBB::tbb
        benchmark::benchmark
        LIB_DEPENDS OptimizationAlgorithms Registration PCRUtilities
)


set(LIBRARY_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)
set(EXECUTABLE_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)