//
// Created by XI on 2023/9/30.
//
// Qt
#include <QFile>
#include <QTextStream>

// STD Includes
#include <chrono>
#include <ctime>
#include <thread>

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
#include <pcl/filters/approximate_voxel_grid.h>
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

TEST(ndt_test, test1)
{
  float posAndRotation[6] = {-0.05, -0.02, 0.09, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
  auto source1 =
      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudSource;
  auto target1 =
      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudTarget;

  std::vector<int> indices_src; // 保存去除的点的索引
  pcl::removeNaNFromPointCloud(*source1, *source1, indices_src);
  std::cout << "remove *cloud_source nan" << endl;

  std::vector<int> indices_tgt; // 保存去除的点的索引
  pcl::removeNaNFromPointCloud(*target1, *target1, indices_tgt);
  std::cout << "remove *cloud_target nan" << endl;

  PointCloudT::Ptr source(new PointCloudT);
  PointCloudT::Ptr target(new PointCloudT);

  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.002, 0.002, 0.002);
  approximate_voxel_filter.setInputCloud(source1);
  approximate_voxel_filter.filter(*source); // 下采样之后的点云保存在filtered_cloud

  approximate_voxel_filter.setInputCloud(target1);
  approximate_voxel_filter.filter(*target); // 下采样之后的点云保存在filtered_cloud

  std::cout << source->size() << std::endl;
  std::cout << target->size() << std::endl;
  TICK(ndt)
  // NDT配准
  // 初始化正太分布NDT对象
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  PointCloudT::Ptr cloud_ndt(new PointCloudT);
  //  ndt.setTransformationEpsilon(0.01);
  ndt.setStepSize(0.0005);
  // 设置ndt网格的分辨率,该尺度与场景最相关，默认为1.0
  ndt.setResolution(0.1);
  ndt.setMaximumIterations(100);

  // 载入点云
  ndt.setInputSource(source);
  ndt.setInputTarget(target);

  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

  ndt.align(*cloud_ndt);

  TOCK(ndt)
  std::cout << "final-ndt " << ndt.getFitnessScore() << std::endl;
  showTrans(source, target, cloud_ndt);
  ASSERT_TRUE(true);
}
using namespace std::chrono_literals;

TEST(ndt, test2)
{
  // Loading first scan of room.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "E:\\CLionProjects\\PointCloudRegistration\\resource\\pcl_resouces\\room_scan1.pcd",
          *target_cloud) == -1) {
    PCL_ERROR("Couldn't read file room_scan1.pcd \n");
  }
  std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;

  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "E:\\CLionProjects\\PointCloudRegistration\\resource\\pcl_resouces\\room_scan2.pcd",
          *input_cloud) == -1) {
    PCL_ERROR("Couldn't read file room_scan2.pcd \n");
  }
  std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud(input_cloud);
  approximate_voxel_filter.filter(*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size()
            << " data points from room_scan2.pcd" << std::endl;

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon(0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize(0.1);
  // Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution(1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations(35);

  // Setting point cloud to be aligned.
  ndt.setInputSource(filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget(target_cloud);

  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align(*output_cloud, init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
            << " score: " << ndt.getFitnessScore() << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);

  // Initializing point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer_final(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer_final->setBackgroundColor(0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0,
                                                                               0);
  viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                                 "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_cloud, 0, 255,
                                                                               0);
  viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                                 "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem(1.0, "global");
  viewer_final->initCameraParameters();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped()) {
    viewer_final->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }
  ASSERT_TRUE(true);
}