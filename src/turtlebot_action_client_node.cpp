#include <ros/ros.h>

#include "turtlebot_action_c.hpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlebot_action_client");
    ros::NodeHandle nodeHandle;
    turtlebot_action_c inst_2(nodeHandle, "tortue");
    ros::spin();
    return 0;
}
