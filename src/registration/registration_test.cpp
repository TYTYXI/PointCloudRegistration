//
// Created by XI on 2022/12/6.
//

#include "gtest/gtest.h"

#include "../dataset/readPlyData.h"

#include <ctime>
#include <pcl/common/common.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "tlboRegistraion.h"

#include "registraion_visualization.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

TEST(tlboRegistraion, test1)
{
  auto tlbo = new tlboRegistraion(readPlyData::testPointCloud()->cloudSource,
                                  readPlyData::testPointCloud()->cloudTarget);
  tlbo->align();

  PointCloudT::Ptr trans(new PointCloudT);

  pcl::transformPointCloud(*readPlyData::testPointCloud()->cloudSource, *trans, tlbo->finalMatrix());

  showTrans(readPlyData::testPointCloud()->cloudSource,
            readPlyData::testPointCloud()->cloudTarget, trans);

  ASSERT_TRUE(true);
}