//
// Created by XI on 2022/12/7.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_REGISTRAION_VISUALIZATION_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_REGISTRAION_VISUALIZATION_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void showTrans(PointCloudT::Ptr cloudSource, PointCloudT::Ptr cloudTarget,
               PointCloudT::Ptr cloudTrans);

#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_REGISTRAION_VISUALIZATION_H_
