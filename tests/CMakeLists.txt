add_ons_test(tests
        SOURCES tlbo_test.cpp
        DEPENDS Eigen3::Eigen)

set(LIBRARY_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)
set(EXECUTABLE_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)