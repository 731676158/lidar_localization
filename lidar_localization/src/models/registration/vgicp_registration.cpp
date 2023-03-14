/*
 * @Description: VGICP 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:45
 */
#include "lidar_localization/models/registration/vgicp_registration.hpp"
//#include "models/registration/tools/fast_vgicp.cpp"

#include "glog/logging.h"
#include <string>

namespace lidar_localization {

VGICPRegistration::VGICPRegistration(const YAML::Node& node)
    :vgicp_ptr_(new fast_gicp::FastVGICP<CloudData::POINT, CloudData::POINT>()) {
    
    float res = node["res"].as<float>();
    int num_threads = node["num_threads"].as<int>();
    float trans_eps = node["trans_eps"].as<float>();
    int num_neighbors = node["num_neighbors"].as<int>();

    SetRegistrationParam(res, num_threads, trans_eps, num_neighbors);
}

VGICPRegistration::VGICPRegistration(float res, int num_threads, float trans_eps, int num_neighbors)
    :vgicp_ptr_(new fast_gicp::FastVGICP<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(res, num_threads, trans_eps, num_neighbors);
}

bool VGICPRegistration::SetRegistrationParam(float res, int num_threads, float trans_eps, int num_neighbors) {
    vgicp_ptr_->setResolution(res);
    vgicp_ptr_->setNumThreads(num_threads);//
    vgicp_ptr_->setTransformationEpsilon(trans_eps);
    if (num_neighbors != 1 && num_neighbors != 7 && num_neighbors != 27) {
        LOG(ERROR) << "number of neighbors is incorrect! Please recheck!"
                   << std::endl
                   << "number can only in 1, 7 and 27." << std::endl;
        return false;
    }
    vgicp_ptr_->setNeighborSearchMethod(
        (num_neighbors == 1) ? settings::NeighborSearchMethod::DIRECT1 :
        (num_neighbors == 7) ? settings::NeighborSearchMethod::DIRECT7 :
        settings::NeighborSearchMethod::DIRECT27);

    LOG(INFO) << "VGICP params: " << std::endl
              << "res: " << res << ", "
              << "trans_eps: " << trans_eps << ", "
              << "num_neighbors: "<< num_neighbors
              << std::endl << std::endl;

    return true;
}

bool VGICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    vgicp_ptr_->setInputTarget(input_target);

    return true;
}

bool VGICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    vgicp_ptr_->setInputSource(input_source);
    vgicp_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = vgicp_ptr_->getFinalTransformation();

    return true;
}
}