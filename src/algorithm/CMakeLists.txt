set(sources algorithm_global.h
        IOABase.cpp IOABase.h
        TeachingLearningBasedOptimization.h
        )

#add_ons_plugin(algorithm
#        SOURCES ${sources}
#        PUBLIC_DEPENDS Eigen3::Eigen TBB::tbb OpenMP::OpenMP_CXX)

add_ons_test(algorithm
        SOURCES algorithm_TEST.cpp ${sources}
        DEPENDS  OpenMP::OpenMP_CXX Eigen3::Eigen TBB::tbb)

#target_compile_definitions(algorithm PRIVATE ALGORITHM_LIB)

set(LIBRARY_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)
set(EXECUTABLE_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)