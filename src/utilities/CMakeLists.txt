set(sources
        UtilitiesHlobal.h
        PCR_Utility.h
        PCR_Utility.cpp
)

add_ons_lib(
        PCRUtilities
        PUBLIC_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}
        INCLUDES ../OptimizationAlgorithms
        SOURCES ${sources}
        DEPENDS Qt::Core ${PCL_LIBRARIES} ${VTK_LIBRARIES} Eigen3::Eigen TBB::tbb
        COMPILE_DEFINITIONS UTILITIES_LIB
)