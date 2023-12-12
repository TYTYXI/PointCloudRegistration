//
// Created by XI on 2022/12/7.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_REGISTRATION_VISUALIZATION_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_REGISTRATION_VISUALIZATION_H_

#include "registration_global.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

REGISTRATION_EXPORT void showTrans(PointCloudT::Ptr cloudSource, PointCloudT::Ptr cloudTarget,
                                   PointCloudT::Ptr cloudTrans);

REGISTRATION_EXPORT void showTrans(PointCloudT::Ptr cloudSource, PointCloudT::Ptr cloudTarget);

#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_REGISTRATION_VISUALIZATION_H_
