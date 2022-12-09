//
// Created by XI on 2022/12/7.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_REGISTRATIONOBJECT_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_REGISTRATIONOBJECT_H_

#include "registration_global.h"

#include <pcl/point_cloud.h>

template <typename PointT>
class REGISTRATION_EXPORT registrationObject
{
  typedef PointT PointType;
  typedef std::shared_ptr<const pcl::PointCloud<PointT>> ConstPtr;


};

#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_REGISTRATIONOBJECT_H_
