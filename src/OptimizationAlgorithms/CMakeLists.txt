set(sources
        OptimizationAlgorithmsGlobal.h
        Types.h Constants.hpp
        Algorithm.hpp Algorithm.cpp
        Population.hpp Population.cpp
        PopulationMatrixType.cpp PopulationMatrixType.hpp
        Problems.hpp Problems.cpp
)

if (CMAKE_CXX_STANDARD EQUAL 17)
    list(APPEND OA_COMPILE_DEF OA_CXX17)
elseif (CMAKE_CXX_STANDARD EQUAL 20)
    list(APPEND OA_COMPILE_DEF OA_CXX20)
endif ()

list(APPEND OA_COMPILE_DEF ALGORITHM_LIB)

add_ons_lib(OptimizationAlgorithms
        INCLUDES ../utilities
        PUBLIC_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}
        SOURCES ${sources}
        PUBLIC_INCLUDES problems
        DEPENDS Eigen3::Eigen TBB::tbb PCRUtilities
        COMPILE_DEFINITIONS ${OA_COMPILE_DEF}
)

add_subdirectory(algorithm)
add_subdirectory(problems)
add_subdirectory(tests)
add_subdirectory(benchmark)

set(LIBRARY_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)
set(EXECUTABLE_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)