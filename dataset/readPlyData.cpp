//
// Created by XI on 2022/11/23.
//

#include "readPlyData.h"

#include <pcl/console/time.h> // TicToc
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <ctime>
#include <random>

static bool is_files_read = false;

static QStringList plyFiles = {};

QStringList readPlyData::plyFilesNames()
{
  plyFiles << R"(./resource/drill/drill_shaft_vrip.ply)"
      //      << R"(./resource/Armadillo/Armadillo.ply)"
      //           << R"(./resource/bunny/bun_zipper_res2.ply)"
      //           <<
      //           R"(./resource/dragon_recon/dragon_vrip_res3.ply)"
      //           <<
      //           R"(./resource/happy_recon/happy_vrip_res3.ply)"
      ;
  return plyFiles;
}

QList<readPlyData::plyPointCloud*> readPlyData::pointClouds()
{
  QList<plyPointCloud*> pointClouds;
  if (!is_files_read) {
    std::uniform_real_distribution<float> u(-M_PI, M_PI);
    std::uniform_real_distribution<float> u2(-2, 2);
    std::default_random_engine e(time(nullptr));

    for (const auto& kfile : (plyFilesNames())) {
      PointCloudT::Ptr cloudSource(new PointCloudT);
      PointCloudT::Ptr cloudTarget(new PointCloudT);
      pcl::io::loadPLYFile(kfile.toStdString(), *cloudSource);

      Eigen::Affine3f transformationMatrix = Eigen::Affine3f::Identity();

      transformationMatrix.translation() << u2(e), u2(e), u2(e);
      //  transformation_matrix.translation() << 0.00, -0.00, -0.00;
      transformationMatrix.rotate(Eigen::AngleAxisf(u(e), Eigen::Vector3f::UnitX()));
      transformationMatrix.rotate(Eigen::AngleAxisf(u(e), Eigen::Vector3f::UnitY()));
      transformationMatrix.rotate(Eigen::AngleAxisf(u(e), Eigen::Vector3f::UnitZ()));

      pcl::transformPointCloud(*cloudSource, *cloudTarget, transformationMatrix);

      auto pointCloud = new plyPointCloud;
      pointCloud->transMatrix = transformationMatrix.matrix();
      pointCloud->cloudSource = cloudSource;
      pointCloud->cloudTarget = cloudTarget;
      pointClouds.emplace_back(pointCloud);
    }

    is_files_read = true;
  }
  return pointClouds;
}

static bool testPointCloudFlag = false;
static readPlyData::plyPointCloud* testPointCloudPtr = nullptr;

readPlyData::plyPointCloud* readPlyData::testPointCloud()
{
  if (!testPointCloudFlag) {
    PointCloudT::Ptr cloudSource(new PointCloudT);
    PointCloudT::Ptr cloudTarget(new PointCloudT);
    pcl::io::loadPLYFile("./resource/monkey/monkey.ply", *cloudSource);

    Eigen::Affine3f transformationMatrix = Eigen::Affine3f::Identity();

    transformationMatrix.translation() << -0.5, -0.2, 0.9;
    transformationMatrix.rotate(Eigen::AngleAxisf(2. / 3. * M_PI, Eigen::Vector3f::UnitX()));
    transformationMatrix.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY()));
    transformationMatrix.rotate(Eigen::AngleAxisf(M_PI / 3, Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud(*cloudSource, *cloudTarget, transformationMatrix);

    testPointCloudPtr = new plyPointCloud;
    testPointCloudPtr->transMatrix = transformationMatrix.matrix();
    testPointCloudPtr->cloudSource = cloudSource;
    testPointCloudPtr->cloudTarget = cloudTarget;
    testPointCloudFlag = true;
  }

  return testPointCloudPtr;
}
