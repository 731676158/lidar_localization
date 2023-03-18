/*
 * @Description: voxel filter 模块实现
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:53:20
 */
#include "lidar_localization/models/cloud_filter/voxel_feature_filter.hpp"

#include "glog/logging.h"

namespace lidar_localization {

VoxelFeatureFilter::VoxelFeatureFilter(const YAML::Node& node) {
    float leaf_size_x = node["leaf_size"][0].as<float>();
    float leaf_size_y = node["leaf_size"][1].as<float>();
    float leaf_size_z = node["leaf_size"][2].as<float>();
    std::string feature_type = node["feature_type"].as<std::string>();
    float sample_ratio = node["sample_ratio"].as<float>();
    
    SetFeatureParam(feature_type, sample_ratio);
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFeatureFilter::VoxelFeatureFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z, std::string feature_type, float sample_ratio) {
    SetFeatureParam(feature_type, sample_ratio);
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFeatureFilter::SetFeatureParam(std::string feature_type, float sample_ratio) {

    voxel_feature_filter_.setRemainFeature(true);

    if (feature_type == "NormalSpace") 
    voxel_feature_filter_.setRemainType(pcl::VoxelGridFeature<CloudData::POINT>::FeatureFilter::NormalSpace);
    else if (feature_type == "StatisiticalOutlier") 
    voxel_feature_filter_.setRemainType(pcl::VoxelGridFeature<CloudData::POINT>::FeatureFilter::StatisiticalOutlier);
    else if (feature_type == "Covariance") 
    voxel_feature_filter_.setRemainType(pcl::VoxelGridFeature<CloudData::POINT>::FeatureFilter::Covariance);

    voxel_feature_filter_.setSampleRatio(sample_ratio);

    std::cout << "Voxel Filter 的 feature 参数为：" << std::endl
              << feature_type << ", "
              << sample_ratio
              << std::endl << std::endl;

    return true;
}

bool VoxelFeatureFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    voxel_feature_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

    std::cout << "Voxel Filter 的参数为：" << std::endl
              << leaf_size_x << ", "
              << leaf_size_y << ", "
              << leaf_size_z 
              << std::endl << std::endl;

    return true;
}

bool VoxelFeatureFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    voxel_feature_filter_.setInputCloud(input_cloud_ptr);
    voxel_feature_filter_.filter(*filtered_cloud_ptr);

    return true;
}
} 