/*
 * @Description: 地图匹配任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/matching/matching_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"
#include "scan_context/Scancontext.h"
using namespace std;

namespace lidar_localization {
MatchingFlow::MatchingFlow(ros::NodeHandle& nh) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);

    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "/map", "/lidar", 100);
    laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/vehicle_link");

    matching_ptr_ = std::make_shared<Matching>();
}

bool MatchingFlow::Run() {
    if (matching_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
        matching_ptr_->GetGlobalMap(global_map_ptr);
        global_map_pub_ptr_->Publish(global_map_ptr);
    }

    if (matching_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
        local_map_pub_ptr_->Publish(matching_ptr_->GetLocalMap());

    ReadData();

    while(HasData()) {
        if (!ValidData())
            continue;

        if (UpdateMatching()) {
            PublishData();
        }
    }

    return true;
}

bool MatchingFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    return true;
}

bool MatchingFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    
    if (matching_ptr_->HasInited())
        return true;
    
    if (gnss_data_buff_.size() == 0)
        return false;
    
    return true;
}

bool MatchingFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    if (matching_ptr_->HasInited()) {
        cloud_data_buff_.pop_front();              //如果已经更新了就取第一帧
        gnss_data_buff_.clear();
        return true;
    }
//这是原来的用GPS初始化的部分，现在改为查询sc初始化
//    current_gnss_data_ = gnss_data_buff_.front();
//
//    double diff_time = current_cloud_data_.time - current_gnss_data_.time;
//    if (diff_time < -0.05) {
//        cloud_data_buff_.pop_front();
//        return false;
//    }
//
//    if (diff_time > 0.05) {
//        gnss_data_buff_.pop_front();
//        return false;
//    }
//
//    cloud_data_buff_.pop_front();
//    gnss_data_buff_.pop_front();

    cloud_data_buff_.pop_front();
//查询sc得到初始位姿
    string config_file_path="/home/sxy/catkin_sl/src/lidar_localization/config/matching/scpara.yaml";
    YAML::Node config_node=YAML::LoadFile(config_file_path);
    string scpath=config_node["scpath"].as<string>();
    string posepath=config_node["posepath"].as<string>();
    SCManager scmanager_read,scmanager_search;
    vector<Eigen::MatrixXd> pose_read;
    ifstream scfile(scpath);
    ifstream posefile(posepath);
    boost::archive::binary_iarchive sc(scfile);
    boost::archive::binary_iarchive pose(posefile);
    sc>>scmanager_read;                            //读取保存在文件中的sc
    pose>>pose_read;                               //读取保存在文件中的位姿

    scmanager_search.polarcontexts_=scmanager_read.polarcontexts_;                              //因为只保存了sc类中这几个成员变量，所以要再初始化一个sc变量
    scmanager_search.polarcontext_invkeys_=scmanager_read.polarcontext_invkeys_;
    scmanager_search.polarcontext_invkeys_mat_=scmanager_read.polarcontext_invkeys_mat_;
    scmanager_search.polarcontext_vkeys_=scmanager_read.polarcontext_vkeys_;

    std::pair<int,float> result=scmanager_search.relocalization(*(current_cloud_data_.cloud_ptr));     //找出初始帧与储存的sc中那一帧相对应
    if(result.first==-1)             //如果没找到，就返回错误
    {
        return false;
    }

    Eigen::Affine3f yaw_trans = pcl::getTransformation(0, 0, 0,  0, 0,yawDiffRad);
    Eigen::Matrix4f yaw_matrix = yaw_trans.matrix();
    init_pose_sc=(pose_read[result.first]).cast<float>();
    init_pose_sc=yaw_matrix*init_pose_sc;

    return true;
}

bool MatchingFlow::UpdateMatching() {
    if (!matching_ptr_->HasInited()) {
//        matching_ptr_->SetGNSSPose(current_gnss_data_.pose);
        matching_ptr_->SetGNSSPose(init_pose_sc);
    }

    return matching_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool MatchingFlow::PublishData() {
    laser_tf_pub_ptr_->SendTransform(laser_odometry_, current_cloud_data_.time);
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);
    current_scan_pub_ptr_->Publish(matching_ptr_->GetCurrentScan());

    return true;
}
}