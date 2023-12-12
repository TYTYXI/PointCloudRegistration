//
// Created by XI on 2022/11/23.
//

#include "ReadPlyData.h"

#include <pcl/console/time.h> // TicToc
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <ctime>
#include <random>

static readPlyData* m_instance = nullptr;
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

QList<readPlyData::plyPointCloud<PointT>*> readPlyData::pointClouds()
{
  QList<plyPointCloud<PointT>*> pointClouds;
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

      auto pointCloud = new plyPointCloud<PointT>;
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
static readPlyData::plyPointCloud<PointT>* testPointCloudPtr = nullptr;

readPlyData::plyPointCloud<PointT>* readPlyData::testPointCloud()
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

    testPointCloudPtr = new plyPointCloud<PointT>;
    testPointCloudPtr->transMatrix = transformationMatrix.matrix();
    testPointCloudPtr->cloudSource = cloudSource;
    testPointCloudPtr->cloudTarget = cloudTarget;
    testPointCloudFlag = true;
  }

  return testPointCloudPtr;
}

readPlyData::readPlyData()
{
  fileNames_.insert(std::make_pair(readPlyData::ARMADILLO, "./resource/Armadillo/Armadillo.ply"));
  fileNames_.insert(std::make_pair(readPlyData::BUNNY, "./resource/bunny/bun_zipper.ply"));
  fileNames_.insert(
      std::make_pair(readPlyData::BUN_ZIPPER_RES_3, "./resource/bunny/bun_zipper_res3.ply"));
  fileNames_.insert(
      std::make_pair(readPlyData::DRAGON_RECON, "./resource/dragon_recon/dragon_vrip.ply"));
  fileNames_.insert(
      std::make_pair(readPlyData::DRAGON_RECON_RES2, "./resource/dragon_recon/dragon_vrip_res2.ply"));
  fileNames_.insert(
      std::make_pair(readPlyData::HAPPY_RECON, "./resource/happy_recon/happy_vrip.ply"));
  fileNames_.insert(std::make_pair(readPlyData::LUCY, "./resource/lucy/lucy.ply"));
  fileNames_.insert(
      std::make_pair(readPlyData::XYZRGB_DRAGON, "./resource/xyzrgb_dragon/xyzrgb_dragon.ply"));
  fileNames_.insert(std::make_pair(readPlyData::XYZRGB_MANUSCRIPT,
                                   "./resource/xyzrgb_manuscript/xyzrgb_manuscript.ply"));
  fileNames_.insert(std::make_pair(readPlyData::XYZRGB_STATUETTE,
                                   "./resource/xyzrgb_statuette/xyzrgb_statuette.ply"));
  fileNames_.insert(
      std::make_pair(readPlyData::HAND, "./resource/LargeGeometricModelsArchive/hand.ply"));
}

readPlyData::~readPlyData()
{
  delete m_instance;
  m_instance = nullptr;
}

readPlyData* readPlyData::instance()
{
  if (!m_instance) {
    m_instance = new readPlyData;
  }
  return m_instance;
}

readPlyData::plyPointCloud<PointT>* readPlyData::testPointCloud(readPlyData::PlyFileName name,
                                                                float posAndRotation[6])
{
  if (auto it = testClouds_.find(name); it != testClouds_.end()) {

    Eigen::Affine3f transformationMatrix = Eigen::Affine3f::Identity();

    transformationMatrix.translation() << posAndRotation[0], posAndRotation[1], posAndRotation[2];
    transformationMatrix.rotate(Eigen::AngleAxisf(posAndRotation[3], Eigen::Vector3f::UnitX()));
    transformationMatrix.rotate(Eigen::AngleAxisf(posAndRotation[4], Eigen::Vector3f::UnitY()));
    transformationMatrix.rotate(Eigen::AngleAxisf(posAndRotation[5], Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*((*it).second->cloudSource), *((*it).second->cloudTarget),
                             transformationMatrix);

    return it->second;
  }

  PointCloudT::Ptr cloudSource(new PointCloudT);
  PointCloudT::Ptr cloudTarget(new PointCloudT);
  pcl::io::loadPLYFile(fileNames_[name].toStdString(), *cloudSource);

  Eigen::Affine3f transformationMatrix = Eigen::Affine3f::Identity();

  transformationMatrix.translation() << posAndRotation[0], posAndRotation[1], posAndRotation[2];
  transformationMatrix.rotate(Eigen::AngleAxisf(posAndRotation[3], Eigen::Vector3f::UnitX()));
  transformationMatrix.rotate(Eigen::AngleAxisf(posAndRotation[4], Eigen::Vector3f::UnitY()));
  transformationMatrix.rotate(Eigen::AngleAxisf(posAndRotation[5], Eigen::Vector3f::UnitZ()));
  //  transformationMatrix.translation() << -0.5, -0.2, 0.9;
  //  transformationMatrix.rotate(Eigen::AngleAxisf(2. / 3. * M_PI, Eigen::Vector3f::UnitX()));
  //  transformationMatrix.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY()));
  //  transformationMatrix.rotate(Eigen::AngleAxisf(M_PI / 3, Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud(*cloudSource, *cloudTarget, transformationMatrix);

  auto pointCloudPtr = new plyPointCloud<PointT>;
  pointCloudPtr->transMatrix = transformationMatrix.matrix();
  pointCloudPtr->cloudSource = cloudSource;
  pointCloudPtr->cloudTarget = cloudTarget;

  testClouds_[name] = pointCloudPtr;
  return pointCloudPtr;
}

std::map<readPlyData::PlyFileName, QString> readPlyData::fileNames() const
{
  return fileNames_;
}
