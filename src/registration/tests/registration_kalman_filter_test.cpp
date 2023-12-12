//
// Created by XI on 2023/10/16.
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
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <PCR_Utility.h>
#include <PointCloudKF.h>

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

// filter Includes
#include "filter/FusionEKF.h"

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

TEST(navi_km_test, test1)
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

  PointCloudKF fusionEKF((*cloud)[0].x, (*cloud)[0].y, (*cloud)[0].z);

  int index = 0;

  PointCloudT::Ptr icp_trans(new PointCloudT);
  for (size_t l = 1; l < cloud->size(); ++l) {

    MeasurementPackage meas_package;
    meas_package.sensor_type_ = MeasurementPackage::ELECTRONIC;
    meas_package.raw_measurements_ = Eigen::VectorXd(3);
    meas_package.raw_measurements_ << (*cloud)[l].x, (*cloud)[l].y, (*cloud)[l].z;
    meas_package.timestamp_ = 25 * 1000 * index++;

    fusionEKF.ProcessMeasurement(meas_package);

    double p_x = fusionEKF.ekf_.x_(0);
    double p_y = fusionEKF.ekf_.x_(1);
    double p_z = fusionEKF.ekf_.x_(2);
    (*icp_trans).emplace_back(pcl::PointXYZ(p_x, p_y, p_z));
  }

  std::cout<<icp_trans->size()<<std::endl;
  showTrans(cloud, cloudTarget, icp_trans);
  ASSERT_TRUE(true);
}
