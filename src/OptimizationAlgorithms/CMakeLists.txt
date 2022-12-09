set(sources
        OptimizationAlgorithmsGlobal.h
        Types.h Constants.hpp
        Algorithm.hpp Algorithm.cpp
        Population.hpp Population.cpp
        Problems.hpp Problems.cpp
        algorithm/TeachingLearningBasedOptimization.h algorithm/TeachingLearningBasedOptimization.cpp
        algorithm/MultipleClassTeachingLearningBasedOptimization.h
        algorithm/MultipleClassTeachingLearningBasedOptimization.cpp
        algorithm/ArtificialBeeColony.h
        algorithm/AlgorithmObject.h
        algorithm/AlgorithmObject.cpp

        problems/test1Problem.cpp problems/test1Problem.h
        problems/test2Problem.cpp problems/test2Problem.h
        problems/NullProblem.cpp problems/NullProblem.h
        )

add_ons_plugin(OptimizationAlgorithms
        SOURCES ${sources}
        PUBLIC_DEPENDS Eigen3::Eigen TBB::tbb)

add_ons_test(OptimizationAlgorithms
        SOURCES tests/OATest.cpp
        DEPENDS OptimizationAlgorithms Eigen3::Eigen TBB::tbb)

add_ons_test(OptimizationAlgorithms_benchmark
        SOURCES tests/OABenchmark.cpp
        DEPENDS OptimizationAlgorithms Eigen3::Eigen TBB::tbb)

#target_compile_definitions(algorithm PRIVATE ALGORITHM_LIB)

if (CMAKE_CXX_STANDARD EQUAL 17)
    add_compile_definitions(OA_CXX17)
elseif (CMAKE_CXX_STANDARD EQUAL 20)
    add_compile_definitions(OA_CXX20)
endif ()

set(LIBRARY_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)
set(EXECUTABLE_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)