/*
 * @Description: voxel filter 模块
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:37:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FEATURE_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FEATURE_FILTER_HPP_

#include "voxel_grid_feature.h"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization {
class VoxelFeatureFilter: public CloudFilterInterface {
  public:
    VoxelFeatureFilter(const YAML::Node& node);
    VoxelFeatureFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z, std::string feature_type, float sample_ratio);

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    bool SetFeatureParam(std::string feature_type, float sample_ratio);

  private:
    pcl::VoxelGridFeature<CloudData::POINT> voxel_feature_filter_;
};
}
#endif