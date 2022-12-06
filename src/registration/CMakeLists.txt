set(sources
        ../algorithm/algorithm_global.h
        ../algorithm/IOABase.cpp ../algorithm/IOABase.h
        ../algorithm/TeachingLearningBasedOptimization.h
        ../algorithm/MultipleClassTeachingLearningBasedOptimization.h
        ../algorithm/ArtificialBeeColony.h
        registration_global.h
        tlboRegistraion.h tlboRegistraion.cpp
        )

add_ons_plugin(registration
        SOURCES ${sources}
        PUBLIC_DEPENDS Qt::Core ${PCL_LIBRARIES}  Eigen3::Eigen TBB::tbb OpenMP::OpenMP_CXX)

add_ons_test(registration
        SOURCES registration_test.cpp ${sources}
        DEPENDS Qt::Core registration dataset ${PCL_LIBRARIES}  OpenMP::OpenMP_CXX Eigen3::Eigen TBB::tbb)

add_ons_test(registration_benchmark
        SOURCES registration_benchmark.cpp ${sources}
        DEPENDS Qt::Core registration dataset ${PCL_LIBRARIES}  OpenMP::OpenMP_CXX Eigen3::Eigen TBB::tbb)

#target_compile_definitions(algorithm PRIVATE ALGORITHM_LIB)

set(LIBRARY_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)
set(EXECUTABLE_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)