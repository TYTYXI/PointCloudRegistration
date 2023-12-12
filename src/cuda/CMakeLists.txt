project(demo)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)#设置编译后的程序存放位置
set(SRC_LIST cuda_test.cpp foo.cu foo.cuh)#添加src目录下的所有源文件
include_directories(include)#添加头文件路径
add_executable(${PROJECT_NAME} ${SRC_LIST})