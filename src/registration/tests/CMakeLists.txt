set(sources
        registration_ia_ransac_test.cpp
        registration_ndt_test.cpp
        registration_tradition_test.cpp
        registration_icp_p2n_test.cpp
        registration_icp_p2p_test.cpp
        registration_icp_n2n_test.cpp
        registration_ia_fpcs_test.cpp
        registration_ia_kfpcs_test.cpp
        registration_cpd_test.cpp
        registration_navi_test.cpp
        registration_test.cpp
        registration_kalman_filter_test.cpp
)

add_ons_test(
        Registration
        SOURCES ${sources}
        DEPENDS Qt::Core dataset ${PCL_LIBRARIES} ${VTK_LIBRARIES} Eigen3::Eigen TBB::tbb
        OpenMP::OpenMP_CXX
        LIB_DEPENDS OptimizationAlgorithms Registration PCRUtilities
)

set(LIBRARY_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)
set(EXECUTABLE_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)