//
// Created by XI on 2022/11/23.
//

#ifndef POINTCLOUDREGISTRATION_DATASET_READPLYDATA_H_
#define POINTCLOUDREGISTRATION_DATASET_READPLYDATA_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QString>
#include <QStringList>
#include <QtGlobal>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Q_DECL_EXPORT readPlyData
{
public:
  struct plyPointCloud
  {
  public:
    PointCloudT::Ptr cloudSource;
    PointCloudT::Ptr cloudTarget;

    Eigen::Matrix4f transMatrix;
  };

  QStringList plyFilesNames();

  QList<plyPointCloud*> pointClouds();

  static plyPointCloud* testPointCloud();
};

#endif // POINTCLOUDREGISTRATION_DATASET_READPLYDATA_H_
