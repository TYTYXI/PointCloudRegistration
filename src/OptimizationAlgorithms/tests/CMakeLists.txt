set(sources
        OATest.cpp
        WOATest.cpp
        TLBOTest.cpp
        MCTLBOTest.cpp
        TLBOMatrixTest.cpp
        TLBWOATest.cpp
        MWGTLBWOTest.cpp
)

add_ons_test(
        OptimizationAlgorithms
        SOURCES ${sources}
        DEPENDS Eigen3::Eigen TBB::tbb
        LIB_DEPENDS OptimizationAlgorithms PCRUtilities
)
target_compile_definitions(OptimizationAlgorithms PRIVATE NOMINMAX)
set(LIBRARY_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)
set(EXECUTABLE_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)