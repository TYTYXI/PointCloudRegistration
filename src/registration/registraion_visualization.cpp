//
// Created by XI on 2022/12/7.
//

#include "registraion_visualization.h"

void showTrans(PointCloudT::Ptr cloudSource, PointCloudT::Ptr cloudTarget,
               PointCloudT::Ptr cloudTrans)
{
  pcl::visualization::PCLVisualizer viewer("demo");
  // Create two vertically separated viewports
  int v1(0);
  int v2(1);
  //  viewer.createViewPort(0.0, 0.0, 1.0/3.0, 1.0, v1);
  //  viewer.createViewPort(1.0/3.0, 0.0, 2.0/3.0, 1.0, v2);
  //  viewer.createViewPort(2.0/3.0, 0.0, 1.0, 1.0, v3);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  // The color we will be using
  float bckgr_gray_level = 0.0; // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;
  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(
      cloudTarget, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
      (int)255 * txt_gray_lvl);
  viewer.addPointCloud(cloudTarget, cloud_in_color_h, "1", v1);
  viewer.addPointCloud(cloudTarget, cloud_in_color_h, "2", v2);
  //  viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v3", v3);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloudTrans, 20, 180, 20);
  viewer.addPointCloud(cloudSource, cloud_tr_color_h, "3", v1);
  viewer.addPointCloud(cloudTrans, cloud_tr_color_h, "4", v2);

  // Set background color
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation
  viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize(1280, 1024); // Visualiser window size

  // Display the visualiser
  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }
}
