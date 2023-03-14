/*
 * @Description: VGICP 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_VGICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_VGICP_REGISTRATION_HPP_

#include "lidar_localization/models/registration/tools/fast_vgicp.hpp"
#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {
class VGICPRegistration: public RegistrationInterface {
  public:
    VGICPRegistration(const YAML::Node& node);
    VGICPRegistration(float res, int num_threads, float trans_eps, int num_neighbors);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
  
  private:
    bool SetRegistrationParam(float res, int num_threads, float trans_eps, int num_neighbors);

  private:
    fast_gicp::FastVGICP<CloudData::POINT, CloudData::POINT>::Ptr vgicp_ptr_;
};
}

#endif