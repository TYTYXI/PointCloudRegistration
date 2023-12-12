//
// Created by XI on 2023/9/30.
//
// STD Includes
#include <ctime>

// PCL Includes
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "gtest/gtest.h"

using pcl::NormalEstimation;
using pcl::search::KdTree;
using PointT = pcl::PointXYZ;
typedef pcl::PointCloud<PointT> PointCloud;

// 由旋转平移矩阵计算旋转角度
void matrix2angle(Eigen::Matrix4f& result_trans, Eigen::Vector3f& result_angle)
{
  double ax, ay, az;
  if (result_trans(2, 0) == 1 || result_trans(2, 0) == -1) {
    az = 0;
    double dlta;
    dlta = atan2(result_trans(0, 1), result_trans(0, 2));
    if (result_trans(2, 0) == -1) {
      ay = M_PI / 2;
      ax = az + dlta;
    } else {
      ay = -M_PI / 2;
      ax = -az + dlta;
    }
  } else {
    ay = -asin(result_trans(2, 0));
    ax = atan2(result_trans(2, 1) / cos(ay), result_trans(2, 2) / cos(ay));
    az = atan2(result_trans(1, 0) / cos(ay), result_trans(0, 0) / cos(ay));
  }
  result_angle << ax, ay, az;
}

// dataset Includes
#include "ReadPlyData.h"

// Registration Includes
#include "registration_visualization.h"

TEST(ia_ransac, test1)
{
  // 加载点云文件
  float posAndRotation[6] = {-0.5, -0.2, 0.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
  auto cloud_src_o =
      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudSource;
  auto cloud_tgt_o =
      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudTarget;
  // 去除NAN点
  std::vector<int> indices_src; // 保存去除的点的索引
  pcl::removeNaNFromPointCloud(*cloud_src_o, *cloud_src_o, indices_src);
  std::cout << "remove *cloud_src_o nan" << endl;
  // 下采样滤波
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(0.2, 0.2, 0.2);
  voxel_grid.setInputCloud(cloud_src_o);
  PointCloud::Ptr cloud_src(new PointCloud);
  voxel_grid.filter(*cloud_src);
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter; // 创建滤波器对象
  pcFilter.setInputCloud(cloud_src_o);               // 设置待滤波的点云
  pcFilter.setRadiusSearch(0.8);                     // 设置搜索半径
  pcFilter.setMinNeighborsInRadius(2);               // 设置一个内点最少的邻居数目
  pcFilter.filter(*cloud_src);                       // 滤波结果存储到cloud_filtered
  pcl::UniformSampling<pcl::PointXYZ> US;            // 创建均匀采样滤波对象
  US.setInputCloud(cloud_src_o);                     // 设置输入点云
  US.setRadiusSearch(0.5f);                          // 设置滤波时创建球体的半径
  US.filter(*cloud_src); // 执行滤波并将结果保存在cloud_filtered中
  std::cout << "down size *cloud_src_o from " << cloud_src_o->size() << "to" << cloud_src->size()
            << endl;
  //  pcl::io::savePCDFileASCII("bunny_src_down.pcd", *cloud_src);
  // 计算表面法线
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
  ne_src.setInputCloud(cloud_src);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>());
  ne_src.setSearchMethod(tree_src);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud<pcl::Normal>);
  ne_src.setRadiusSearch(0.02);
  ne_src.compute(*cloud_src_normals);

  std::vector<int> indices_tgt;
  pcl::removeNaNFromPointCloud(*cloud_tgt_o, *cloud_tgt_o, indices_tgt);
  std::cout << "remove *cloud_tgt_o nan" << endl;

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
  voxel_grid_2.setLeafSize(0.001, 0.001, 0.0001);
  voxel_grid_2.setInputCloud(cloud_tgt_o);
  PointCloud::Ptr cloud_tgt(new PointCloud);
  voxel_grid_2.filter(*cloud_tgt);
  std::cout << "down size *cloud_tgt_o.pcd from " << cloud_tgt_o->size() << "to"
            << cloud_tgt->size() << endl;
  //  pcl::io::savePCDFileASCII("bunny_tgt_down.pcd", *cloud_tgt);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
  ne_tgt.setInputCloud(cloud_tgt);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree<pcl::PointXYZ>());
  ne_tgt.setSearchMethod(tree_tgt);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud<pcl::Normal>);
  // ne_tgt.setKSearch(20);
  ne_tgt.setRadiusSearch(0.02);
  ne_tgt.compute(*cloud_tgt_normals);

  // 计算FPFH
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
  fpfh_src.setInputCloud(cloud_src);
  fpfh_src.setInputNormals(cloud_src_normals);
  pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
  fpfh_src.setSearchMethod(tree_src_fpfh);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
  fpfh_src.setRadiusSearch(0.05);
  fpfh_src.compute(*fpfhs_src);
  std::cout << "compute *cloud_src fpfh" << endl;

  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
  fpfh_tgt.setInputCloud(cloud_tgt);
  fpfh_tgt.setInputNormals(cloud_tgt_normals);
  pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
  fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
  fpfh_tgt.setRadiusSearch(0.05);
  fpfh_tgt.compute(*fpfhs_tgt);
  std::cout << "compute *cloud_tgt fpfh" << endl;

  // SAC配准
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
  scia.setInputSource(cloud_src);
  scia.setInputTarget(cloud_tgt);
  scia.setSourceFeatures(fpfhs_src);
  scia.setTargetFeatures(fpfhs_tgt);
  scia.setMaximumIterations(300);
  // scia.setMinSampleDistance(1);
  // scia.setNumberOfSamples(2);
  // scia.setCorrespondenceRandomness(20);
  PointCloud::Ptr sac_result(new PointCloud);
  scia.align(*sac_result);
  std::cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore()
            << endl;
  Eigen::Matrix4f sac_trans;
  sac_trans = scia.getFinalTransformation();
  std::cout << sac_trans << endl;
  pcl::transformPointCloud(*cloud_src_o, *sac_result, sac_trans);
  //  pcl::io::savePCDFileASCII("bunny_transformed_sac.pcd", *sac_result);

  // icp配准
  PointCloud::Ptr icp_result(new PointCloud);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_src);
  icp.setInputTarget(cloud_tgt_o);
  //  showTrans(cloud_src_o, cloud_tgt_o,sac_result);
  // Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be
  // ignored)
  //  icp.setMaxCorrespondenceDistance(0.04);
  // 最大迭代次数
  icp.setMaximumIterations(1000);
  // 两次变化矩阵之间的差值
  //  icp.setTransformationEpsilon(1e-10);
  // 均方误差
  //  icp.setEuclideanFitnessEpsilon(0.2);
  icp.align(*icp_result, sac_trans);

  std::cout << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore()
            << std::endl;
  Eigen::Matrix4f icp_trans;
  icp_trans = icp.getFinalTransformation();
  // cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
  std::cout << icp_trans << endl;
  // 使用创建的变换对未过滤的输入点云进行变换
  pcl::transformPointCloud(*cloud_src, *icp_result, icp_trans);
  // 保存转换的输入点云
  //  pcl::io::savePCDFileASCII("bunny_transformed_sac_ndt.pcd", *icp_result);

  // 计算误差
  Eigen::Vector3f ANGLE_origin;
  ANGLE_origin << 0, 0, M_PI / 5;
  double error_x, error_y, error_z;
  Eigen::Vector3f ANGLE_result;
  matrix2angle(icp_trans, ANGLE_result);
  error_x = fabs(ANGLE_result(0)) - fabs(ANGLE_origin(0));
  error_y = fabs(ANGLE_result(1)) - fabs(ANGLE_origin(1));
  error_z = fabs(ANGLE_result(2)) - fabs(ANGLE_origin(2));
  cout << "original angle in x y z:\n" << ANGLE_origin << endl;
  cout << "error in aixs_x: " << error_x << "  error in aixs_y: " << error_y
       << "  error in aixs_z: " << error_z << endl;

  // 可视化
  showTrans(cloud_src, cloud_tgt, icp_result);

  ASSERT_TRUE(true);
}
