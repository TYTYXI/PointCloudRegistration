#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_L2_PE_GPU_CUH_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_L2_PE_GPU_CUH_

// PCL Includes
#include <pcl/point_types.h>
#include <pcl/gpu/containers/device_array.h>

extern "C" float cloud2GPU(pcl::gpu::DeviceArray<pcl::PointXYZ>& input,
                           pcl::gpu::DeviceArray<pcl::PointXYZ>& target, float& fitnessScore,
                           std::vector<int>& inputIndex, std::vector<int>& targetIndex);

#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_L2_PE_GPU_CUH_
