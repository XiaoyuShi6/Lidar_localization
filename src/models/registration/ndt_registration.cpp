/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:45
 */
#include "lidar_localization/models/registration/ndt_registration.hpp"


namespace lidar_localization {

NDTRegistration::NDTRegistration(const YAML::Node& node)
    :ndt_omp_ptr(new pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {

    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();
    int NumThreads=node["NumThread"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter,NumThreads);
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter,int NumThreads)
    :ndt_omp_ptr(new pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(res, step_size, trans_eps, max_iter,NumThreads);
}

bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter,int NumThreads) {
    ndt_omp_ptr->setResolution(res);
    ndt_omp_ptr->setStepSize(step_size);
    ndt_omp_ptr->setTransformationEpsilon(trans_eps);
    ndt_omp_ptr->setMaximumIterations(max_iter);
    ndt_omp_ptr->setNumThreads(NumThreads);
    ndt_omp_ptr->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    std::cout << "NDT 的匹配参数为：" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter
              << std::endl << std::endl;

    return true;
}

bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    ndt_omp_ptr->setInputTarget(input_target);

    return true;
}

bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source,
                                const Eigen::Matrix4f& predict_pose,
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    ndt_omp_ptr->setInputSource(input_source);
    ndt_omp_ptr->align(*result_cloud_ptr, predict_pose);
    result_pose = ndt_omp_ptr->getFinalTransformation();

    return true;
}

float NDTRegistration::GetFitnessScore() {
    return ndt_omp_ptr->getFitnessScore();
}
}