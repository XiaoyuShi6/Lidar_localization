//
// Created by sxy on 2020/10/21.
//

#include "lidar_localization/scan_context/Scancontext.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include "lidar_localization/sensor_data/cloud_data.hpp"
#include <boost/bind.hpp>

using namespace std;
using namespace lidar_localization;

int makescan_gap=0;
int savescan_gap=0;

SCManager scmanager;
ros::Publisher sc_pub;
void Cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
//void Pose_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);
int main(int argc,char** argv)
{
    ros::init(argc,argv,"makesc");
    ros::NodeHandle nh;
    sc_pub=nh.advertise<sensor_msgs::Image>("sc_topic",100);
    ros::Subscriber cloud_sub=nh.subscribe("/synced_cloud",100000,Cloud_callback);
//    ros::Subscriber cloud_sub=nh.subscribe("/synced_cloud",100000,boost::bind(&Cloud_callback,_1,sc_pub));
//    ros::Subscriber pose_sub=nh.subscribe("/synced_gnss",100000,Pose_callback);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}
void Cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg_ptr,*input);
    scmanager.makeAndSaveScancontextAndKeys(*input);
    cv::Mat sc_color_img=scmanager.get_color();
    cv_bridge::CvImage out_msg;
    out_msg.header.frame_id="/map";
//    double time=cloud_msg_ptr->header.stamp.toSec();
    out_msg.header.stamp=ros::Time::now();
    out_msg.encoding=sensor_msgs::image_encodings::RGB8;
    out_msg.image=sc_color_img;
    sc_pub.publish(out_msg.toImageMsg());
}