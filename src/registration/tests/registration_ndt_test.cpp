//
// Created by XI on 2023/6/2.
//
#include "gtest/gtest.h"
#include <Eigen/Core>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// Align a rigid object to a scene with clutter and occlusions
TEST(usampling_sac_ia, test1)
{
  // Point clouds
  PointCloudT::Ptr object(new PointCloudT);
  PointCloudT::Ptr object_aligned(new PointCloudT);
  PointCloudT::Ptr scene_before_downsampling(new PointCloudT);
  PointCloudT::Ptr scene(new PointCloudT);
  FeatureCloudT::Ptr object_features(new FeatureCloudT);
  FeatureCloudT::Ptr scene_features(new FeatureCloudT);

  // Load object and scene
  pcl::console::print_highlight("Loading point clouds...\n");
  pcl::io::loadPLYFile(R"(E:\CLionProjects\PointCloudRegistration\resource\navi\baseLine-5-31.ply)",
                       *scene_before_downsampling);
  pcl::io::loadPLYFile(
      R"(E:\CLionProjects\PointCloudRegistration\resource\navi\2023-6-2\regis3.ply)", *object);

  // Downsample
  pcl::console::print_highlight("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.005f;
  grid.setLeafSize(leaf, leaf, leaf);
  grid.setInputCloud(object);
  grid.filter(*object);
  grid.setInputCloud(scene_before_downsampling);
  grid.filter(*scene);

  // Estimate normals for scene
  pcl::console::print_highlight("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT, PointNT> nest;
  nest.setRadiusSearch(0.005);
  nest.setInputCloud(scene);
  nest.setSearchSurface(scene_before_downsampling);
  nest.compute(*scene);

  // Estimate features
  pcl::console::print_highlight("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch(0.025);
  fest.setInputCloud(object);
  fest.setInputNormals(object);
  fest.compute(*object_features);
  fest.setInputCloud(scene);
  fest.setInputNormals(scene);
  fest.compute(*scene_features);

  // Perform alignment
  pcl::console::print_highlight("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
  align.setInputSource(object);
  align.setSourceFeatures(object_features);
  align.setInputTarget(scene);
  align.setTargetFeatures(scene_features);
  align.setMaximumIterations(50000); // Number of RANSAC iterations
  align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(5);            // Number of nearest features to use
  align.setSimilarityThreshold(0.95f);             // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
  align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align(*object_aligned);
  }

  if (align.hasConverged()) {
    // Print results
    printf("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation();
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0),
                             transformation(0, 1), transformation(0, 2));
    pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0),
                             transformation(1, 1), transformation(1, 2));
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0),
                             transformation(2, 1), transformation(2, 2));
    pcl::console::print_info("\n");
    pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3),
                             transformation(1, 3), transformation(2, 3));
    pcl::console::print_info("\n");
    pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());

    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0),
                       "object_aligned");
    visu.spin();
  } else {
    pcl::console::print_error("Alignment failed!\n");
  }

  ASSERT_TRUE(true);
}

#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

//TEST(ndt_test, test1)
//{
//  // Loading first scan of room.
//  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//  pcl::io::loadPLYFile(R"(E:\CLionProjects\PointCloudRegistration\resource\navi\baseLine-5-31.ply)",
//                       *target_cloud);
//  pcl::io::loadPLYFile(
//      R"(E:\CLionProjects\PointCloudRegistration\resource\navi\2023-6-2\regis3.ply)", *input_cloud);
//  // Filtering input scan to roughly 10% of original size to increase speed of registration.
////  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
////  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
////  approximate_voxel_filter.setLeafSize(0.02, 0.02, 0.02);
////  approximate_voxel_filter.setInputCloud(input_cloud);
////  approximate_voxel_filter.filter(*filtered_cloud);
////  std::cout << "Filtered cloud contains " << filtered_cloud->size()
////            << " data points from room_scan2.pcd" << std::endl;
//
//  // Initializing Normal Distributions Transform (NDT).
//  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
//
//  // Setting scale dependent NDT parameters
//  // Setting minimum transformation difference for termination condition.
//  ndt.setTransformationEpsilon(0.00001);
//  // Setting maximum step size for More-Thuente line search.
//  ndt.setStepSize(0.01);
//  // Setting Resolution of NDT grid structure (VoxelGridCovariance).
//  ndt.setResolution(0.3);
//
//  // Setting max number of registration iterations.
//  ndt.setMaximumIterations(500);
//
//  // Setting point cloud to be aligned.
//  ndt.setInputSource(input_cloud);
//  // Setting point cloud to be aligned to.
//  ndt.setInputTarget(target_cloud);
//
//  // Set initial alignment estimate found using robot odometry.
//  Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
//  Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
//  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
//
//  // Calculating required rigid transform to align the input cloud to the target cloud.
//  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//  ndt.align(*output_cloud, init_guess);
//
//  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
//            << " score: " << ndt.getFitnessScore() << std::endl;
//
//  // Transforming unfiltered, input cloud using found transform.
//  pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
//
//  // Saving transformed input cloud.
//  pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);
//
//  // Initializing point cloud visualizer
//  pcl::visualization::PCLVisualizer::Ptr viewer_final(
//      new pcl::visualization::PCLVisualizer("3D Viewer"));
//  viewer_final->setBackgroundColor(0, 0, 0);
//
//  // Coloring and visualizing target cloud (red).
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0,
//                                                                               0);
//  viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
//  viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
//                                                 "target cloud");
//
//  // Coloring and visualizing transformed input cloud (green).
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_cloud, 0, 255,
//                                                                               0);
//  viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
//  viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
//                                                 "output cloud");
//
//  // Starting visualizer
//  viewer_final->addCoordinateSystem(1.0, "global");
//  viewer_final->initCameraParameters();
//
//  // Wait until visualizer window is closed.
//  while (!viewer_final->wasStopped()) {
//    viewer_final->spinOnce(100);
//    std::this_thread::sleep_for(100ms);
//  }
//
//  ASSERT_TRUE(true);
//}