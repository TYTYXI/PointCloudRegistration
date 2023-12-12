//
// Created by XI on 2022/11/23.
//

#ifndef POINTCLOUDREGISTRATION_DATASET_READPLYDATA_H_
#define POINTCLOUDREGISTRATION_DATASET_READPLYDATA_H_

#include <map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QString>
#include <QStringList>
#include <QtGlobal>

#include "DatasetGlobal.h"

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class DATASET_EXPORT readPlyData
{
public:
  template <class T>
  struct plyPointCloud
  {
  public:
    typename pcl::PointCloud<T>::Ptr cloudSource;
    typename pcl::PointCloud<T>::Ptr cloudTarget;

    Eigen::Matrix4f transMatrix;
  };

  enum PlyFileName
  {
    ARMADILLO,
    BUNNY,
    BUN_ZIPPER_RES_3,
    DRAGON_RECON,
    DRAGON_RECON_RES2,
    HAPPY_RECON,
    LUCY,
    XYZRGB_DRAGON,
    XYZRGB_MANUSCRIPT,
    XYZRGB_STATUETTE,
    HAND,
  };

  enum PointCloudType
  {
    XYZ,
    NORMAL,
  };

  static readPlyData* instance();

  QStringList plyFilesNames();

  QList<plyPointCloud<PointT>*> pointClouds();

  std::map<PlyFileName, QString> fileNames() const;

  plyPointCloud<PointT>* testPointCloud();

  readPlyData::plyPointCloud<PointT>* testPointCloud(readPlyData::PlyFileName name,
                                                     float posAndRotation[6]);

protected:
  readPlyData();
  ~readPlyData();

  std::map<PlyFileName, QString> fileNames_;
  std::map<PlyFileName, plyPointCloud<PointT>*> testClouds_;
};

#endif // POINTCLOUDREGISTRATION_DATASET_READPLYDATA_H_
