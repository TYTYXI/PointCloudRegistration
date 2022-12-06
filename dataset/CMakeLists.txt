project(dataset)

set(sources readPlyData.cpp readPlyData.h)

add_library(dataset SHARED ${sources})

target_link_libraries(dataset PRIVATE Qt::Core)
target_link_libraries(dataset PRIVATE ${PCL_LIBRARIES})

set(LIBRARY_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)
set(EXECUTABLE_OUTPUT_PATH ${BIN_OUTPUT_PATH}/bin)