#include <ros/ros.h>

#include "turtlebot_action_s.hpp"







int main(int argc, char** argv)



{
  ros::init(argc, argv, "turtlebot_action_server");
  ros::NodeHandle nodeHandle;

  std::string name = argv[1];
    ROS_INFO("%s",name.c_str());
  turtlebot_action_s  inst_1(nodeHandle, name);


  ros::spin();
  return 0;
}
