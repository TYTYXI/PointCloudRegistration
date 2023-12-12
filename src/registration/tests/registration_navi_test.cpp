//
// Created by XI on 2023/10/8.
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
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <PCR_Utility.h>

// OA Includes
#include "L2_PE_Problem.h"
#include "L2_SimpleProblem.h"
#include "MultipleClassTeachingLearningBasedOptimization.h"
#include "MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn.h"
#include "Problems.hpp"
#include "TeachingLearningBasedOptimization.h"
#include "TeachingLearningBasedWhaleOptimiziotn.h"

// dataset Includes
#include "../dataset/ReadPlyData.h"

// Registration Includes
#include "registration_visualization.h"

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

TEST(navi_mctlbo_test, test1)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_target(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  PointCloudT::Ptr cloudTarget(new PointCloudT);
  pcl::io::loadPLYFile(
      R"(E:\CLionProjects\PointCloudRegistration\resource\navi\dog_baseLine_2_1.ply)",
      *cloudTarget);
  pcl::io::loadPLYFile(
      R"(E:\CLionProjects\PointCloudRegistration\resource\navi\2023-9-25\regis-14-46-18.ply)",
      *cloud);
  // Uniform sampling object.
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  filter.filter(*cloud_filtered);
  std::cout << "inputCloud size = " << (*cloud).size() << std::endl;
  std::cout << "filtered size = " << (*cloud_filtered).size() << std::endl;
  //  showTrans(cloud, cloud_filtered);
  vtkNew<vtkPoints> sourcePoints;
  for (size_t i = 0; i < 100; i++) {
    sourcePoints->InsertNextPoint((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
  }

  vtkNew<vtkPoints> targetPoints;
  for (size_t i = 0; i < 100; i++) {
    targetPoints->InsertNextPoint((*cloudTarget)[i].x, (*cloudTarget)[i].y, (*cloudTarget)[i].z);
  }

  vtkNew<vtkLandmarkTransform> landmarkTransform;
  landmarkTransform->SetSourceLandmarks(sourcePoints);
  landmarkTransform->SetTargetLandmarks(targetPoints);
  landmarkTransform->SetModeToRigidBody();
  landmarkTransform->Update(); // should this be here?
  auto initMatrix = landmarkTransform->GetMatrix();
  Eigen::Matrix4f initTarns;
  for (size_t l = 0; l < 4; ++l) {
    for (size_t m = 0; m < 4; ++m) {
      initTarns(l, m) = initMatrix->GetElement(l, m);
    }
  }

  //  PointCloudT::Ptr icp_trans_1(new PointCloudT);
  //  pcl::IterativeClosestPoint<PointT, PointT> icp;
  //  icp.setMaximumIterations(100);
  //  icp.setInputSource(cloud_base_source);
  //  icp.setInputTarget(cloud_base_target);
  //  icp.align(*icp_trans_1);
  //
  //  std::cout << icp.getFitnessScore() << std::endl;
  L2_SimpleProblem probb;
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
  //  pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, initTarns);
  //  showTrans(cloud, cloudTarget, cloud_filtered);
  probb.setInputCloud(cloud_filtered);

  probb.setTargetCloud(cloudTarget);
  oa::Problem prob{probb};
  oa::Population pop{prob, 25};
  oa::multipleClassTeachingLearningBasedOptimization mctlbo(12, 10, 4);
  //  oa::teachingLearningBasedOptimization mctlbo(20);

  auto res = mctlbo.optimize(pop);

  PointCloudT::Ptr trans(new PointCloudT);

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.translation() << res.championDecisionVariables()[3],
      res.championDecisionVariables()[4], res.championDecisionVariables()[5];
  transform_2.rotate(
      Eigen::AngleAxisf(res.championDecisionVariables()[0], Eigen::Vector3f::UnitX()));
  transform_2.rotate(
      Eigen::AngleAxisf(res.championDecisionVariables()[1], Eigen::Vector3f::UnitY()));
  transform_2.rotate(
      Eigen::AngleAxisf(res.championDecisionVariables()[2], Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud(*cloud_filtered, *trans, transform_2);
  std::cout << transform_2.matrix() << std::endl;
  std::cout << cloud_filtered->size() << std::endl;
  std::cout << "final-tlbo " << res.championFitnessScores()[0] << std::endl;

  PointCloudT::Ptr icp_trans(new PointCloudT);
  pcl::IterativeClosestPoint<PointT, PointT> icp1;
  icp1.setMaximumIterations(100);
  icp1.setInputSource(trans);
  icp1.setInputTarget(cloudTarget);
  icp1.align(*icp_trans);
  std::cout << "final-icp " << icp1.getFitnessScore() << std::endl;
  showTrans(cloud, cloudTarget, icp_trans);
  ASSERT_TRUE(true);
}

using PointCloud = pcl::PointCloud<PointT>;

TEST(navi_ia_ransac, test1)
{
  // 加载点云文件
  //  float posAndRotation[6] = {-0.5, -0.2, 0.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
  //  auto cloud_src_o =
  //      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudSource;
  //  auto cloud_tgt_o =
  //      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudTarget;
  clock_t start = clock();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_o(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_o(new pcl::PointCloud<pcl::PointXYZ>);
  //  pcl::io::loadPLYFile(
  //      R"(E:\CLionProjects\PointCloudRegistration\resource\navi\dog_baseLine_2_1.ply)",
  //      *cloud_tgt_o);
  pcl::io::loadPLYFile(R"(E:\CLionProjects\PointCloudRegistration\resource\navi\baseLine-5-31.ply)",
                       *cloud_tgt_o);
  pcl::io::loadPLYFile(
      R"(E:\CLionProjects\PointCloudRegistration\resource\navi\2023-11-14\regis-14-51-23.ply)",
      *cloud_src_o);
  // 去除NAN点
  std::vector<int> indices_src; // 保存去除的点的索引
  pcl::removeNaNFromPointCloud(*cloud_src_o, *cloud_src_o, indices_src);
  std::cout << "remove *cloud_src_o nan" << endl;
  // 下采样滤波
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(0.3, 0.3, 0.3);
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
  US.setRadiusSearch(0.2f);                           // 设置滤波时创建球体的半径
  US.filter(*cloud_src); // 执行滤波并将结果保存在cloud_filtered中
  std::cout << "down size *cloud_src_o from " << cloud_src_o->size() << "to" << cloud_src->size()
            << endl;
  // 计算表面法线
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_src;
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

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_tgt;
  ne_tgt.setInputCloud(cloud_tgt);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree<pcl::PointXYZ>());
  ne_tgt.setSearchMethod(tree_tgt);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud<pcl::Normal>);
  // ne_tgt.setKSearch(20);
  ne_tgt.setRadiusSearch(0.02);
  ne_tgt.compute(*cloud_tgt_normals);

  // 计算FPFH
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
  fpfh_src.setInputCloud(cloud_src);
  fpfh_src.setInputNormals(cloud_src_normals);
  pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
  fpfh_src.setSearchMethod(tree_src_fpfh);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
  fpfh_src.setRadiusSearch(0.5);
  fpfh_src.compute(*fpfhs_src);
  std::cout << "compute *cloud_src fpfh" << endl;

  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
  fpfh_tgt.setInputCloud(cloud_tgt);
  fpfh_tgt.setInputNormals(cloud_tgt_normals);
  pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
  fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
  fpfh_tgt.setRadiusSearch(0.5);
  fpfh_tgt.compute(*fpfhs_tgt);
  std::cout << "compute *cloud_tgt fpfh" << endl;
  // 四点法配准
  PointCloud::Ptr pcs(new PointCloud);
  pcl::registration::KFPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> fpcs;

  fpcs.setInputSource(cloud_src); // 输入待配准点云
  fpcs.setInputTarget(cloud_tgt); // 输入目标点云

  // 参数设置
  fpcs.setApproxOverlap(0.7); // 两点云重叠度
  fpcs.setDelta(0.5);         // Bunny
  // fpcs.setDelta(0.5);//hippo
  fpcs.setMaxComputationTime(50);
  fpcs.setNumberOfSamples(1000);
  fpcs.align(*pcs);
  Eigen::Matrix4f pcsTrans = fpcs.getFinalTransformation();
  std::cout << fpcs.getFitnessScore() << endl;
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
  clock_t sac_time = clock();

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

  clock_t end = clock();
  cout << "total time: " << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << endl;
  // 我把计算法线和点特征直方图的时间也算在SAC里面了
  cout << "sac time: " << (double)(sac_time - start) / (double)CLOCKS_PER_SEC << " s" << endl;
  cout << "icp time: " << (double)(end - sac_time) / (double)CLOCKS_PER_SEC << " s" << endl;

  std::cout << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore()
            << std::endl;
  Eigen::Matrix4f icp_trans;
  icp_trans = icp.getFinalTransformation();
  // cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
  std::cout << icp_trans << endl;
  // 使用创建的变换对未过滤的输入点云进行变换
  pcl::transformPointCloud(*cloud_src, *icp_result, icp_trans);
  // 可视化
  showTrans(cloud_src, cloud_tgt, icp_result);

  ASSERT_TRUE(true);
}
