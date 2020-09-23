
#include <ros/ros.h>

#include "lidar_localization/matching/matching_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {


    ros::init(argc, argv, "matching_node");
    ros::NodeHandle nh;

    std::shared_ptr<MatchingFlow> matching_flow_ptr = std::make_shared<MatchingFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        matching_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}