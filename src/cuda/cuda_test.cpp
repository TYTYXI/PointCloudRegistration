//
// Created by XI on 2023/9/25.
//
#include "foo.cuh"
#include <iostream>
#include <stdio.h>

// #include "opencv2/opencv.hpp"
int main()
{

  // cv::Mat image = cv::imread("./images/P0000.png");
  // unsigned char *array = image.data;
  // 创建数组
  const int length = 37000;
  float a[length], b[length];
  float* c = (float*)malloc(length * sizeof(float));
  for (int i = 0; i < length; i++) {
    a[i] = 1;
    b[i] = 2;
  }
  // 矩阵加法运算
  c = matAdd(a, b, length);

  std::cout << "a" << std::endl;
  // 输出查看是否完成计算
  //  for (size_t i = 0; i < length; ++i) {
  //    for (int j = 0; j < length; j++) {
  //      std::cout << a[j] << " " << b[j] << " " << c[j] << std::endl;
  //    }
  //  }

  return 0;
}
