#ifndef FOO_CUH
#define FOO_CUH

#include <stdio.h>
//非模板函数才可以进行C连接
extern "C" float *matAdd(float *a,float *b,int length);

#endif
