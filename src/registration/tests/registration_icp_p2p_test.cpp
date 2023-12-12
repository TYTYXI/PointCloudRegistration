//
// Created by XI on 2023/10/4.
//
// Qt
#include <QFile>
#include <QTextStream>

// STD Includes
#include <ctime>

// gtest Includes
#include "gtest/gtest.h"

// VTK Includes
#include <vtkLandmarkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkUnstructuredGrid.h>
#include <vtkVertex.h>
#include <vtkVertexGlyphFilter.h>

// PCL Includes
#include <pcl/common/common.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h> //ndt头文件
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

// dataset Includes
#include "../dataset/ReadPlyData.h"

// Utilities Includes
#include "PCR_Utility.h"

// Registration Includes
#include "registration_visualization.h"

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

TEST(icp_p2p_test, test1)
{
  float posAndRotation[6] = {-0.5, -0.2, 0.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
  auto source =
      readPlyData::instance()->testPointCloud(readPlyData::ARMADILLO, posAndRotation)->cloudSource;
  auto target =
      readPlyData::instance()->testPointCloud(readPlyData::ARMADILLO, posAndRotation)->cloudTarget;
  PointCloudT::Ptr icp_trans(new PointCloudT);
  auto begin = std::chrono::high_resolution_clock::now();
  pcl::IterativeClosestPoint<PointT, PointT> icp1;
  icp1.setEuclideanFitnessEpsilon(0.0001);
  icp1.setMaximumIterations(1);
  icp1.setInputSource(source);
  icp1.setInputTarget(target);
  icp1.align(*icp_trans);
  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  //    std::cout << elapsed.count() * 1e-9 << std::endl;
  printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
  std::cout << "final-icp " << icp1.getFitnessScore() << std::endl;
  showTrans(source, target, icp_trans);
  ASSERT_TRUE(true);
}
