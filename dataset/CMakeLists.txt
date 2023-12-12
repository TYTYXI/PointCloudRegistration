set(sources
        DatasetGlobal.h
        ReadPlyData.h
        ReadPlyData.cpp
)

add_ons_lib(dataset
        PUBLIC_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}
        SOURCES ${sources}
        PUBLIC_INCLUDES problems
        DEPENDS Qt::Core ${PCL_LIBRARIES}
        COMPILE_DEFINITIONS DATASET_LIB
)

set(LIBRARY_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)
set(EXECUTABLE_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)