set(sources
        OptimizationAlgorithmsGlobal.h
        Types.h Constants.hpp
        Algorithm.hpp Algorithm.cpp
        Population.hpp Population.cpp
        Problems.hpp Problems.cpp
        algorithm/TeachingLearningBasedOptimization.h algorithm/TeachingLearningBasedOptimization.cpp
        algorithm/MultipleClassTeachingLearningBasedOptimization.h
        algorithm/MultipleClassTeachingLearningBasedOptimization.cpp
        algorithm/CuckooSearch.h algorithm/CuckooSearch.cpp
        algorithm/WhaleOptimization.h algorithm/WhaleOptimization.cpp
        algorithm/ArtificialBeeColony.h
        algorithm/AlgorithmObject.h
        algorithm/AlgorithmObject.cpp

        problems/problems.h
        problems/UnimodalBenchmarkFunctions/UBFs.h
        problems/MultimodalBenchmarkFunctions/MBFs.h
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBFs.h
        problems/test1Problem.cpp problems/test1Problem.h
        problems/test2Problem.cpp problems/test2Problem.h
        problems/NullProblem.cpp problems/NullProblem.h
        problems/UnimodalBenchmarkFunctions/UBF1.cpp problems/UnimodalBenchmarkFunctions/UBF1.h
        problems/UnimodalBenchmarkFunctions/UBF2.cpp problems/UnimodalBenchmarkFunctions/UBF2.h
        problems/UnimodalBenchmarkFunctions/UBF3.cpp problems/UnimodalBenchmarkFunctions/UBF3.h
        problems/UnimodalBenchmarkFunctions/UBF4.cpp problems/UnimodalBenchmarkFunctions/UBF4.h
        problems/UnimodalBenchmarkFunctions/UBF5.cpp problems/UnimodalBenchmarkFunctions/UBF5.h
        problems/UnimodalBenchmarkFunctions/UBF6.cpp problems/UnimodalBenchmarkFunctions/UBF6.h
        problems/UnimodalBenchmarkFunctions/UBF7.cpp problems/UnimodalBenchmarkFunctions/UBF7.h
        problems/MultimodalBenchmarkFunctions/MBF1.h problems/MultimodalBenchmarkFunctions/MBF1.cpp
        problems/MultimodalBenchmarkFunctions/MBF2.h problems/MultimodalBenchmarkFunctions/MBF2.cpp
        problems/MultimodalBenchmarkFunctions/MBF3.h problems/MultimodalBenchmarkFunctions/MBF3.cpp
        problems/MultimodalBenchmarkFunctions/MBF4.h problems/MultimodalBenchmarkFunctions/MBF4.cpp
        problems/MultimodalBenchmarkFunctions/MBF5.h problems/MultimodalBenchmarkFunctions/MBF5.cpp
        problems/MultimodalBenchmarkFunctions/MBF6.h problems/MultimodalBenchmarkFunctions/MBF6.cpp
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF1.h
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF1.cpp
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF2.h
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF2.cpp
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF3.h
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF3.cpp
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF4.h
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF4.cpp
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF5.h
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF5.cpp
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF6.h
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF6.cpp
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF7.h
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF7.cpp
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF8.h
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF8.cpp
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF9.h
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF9.cpp
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF10.h
        problems/Fixed-dimenstionMultimodalBenchmarkFunctions/FMBF10.cpp
        )

add_ons_plugin(OptimizationAlgorithms
        SOURCES ${sources}
        PUBLIC_INCLUDES problems
        PUBLIC_DEPENDS Eigen3::Eigen TBB::tbb)

add_ons_test(OptimizationAlgorithms
        SOURCES tests/OATest.cpp tests/WOATest.cpp tests/TLBOTest.cpp tests/MCTLBO.cpp
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