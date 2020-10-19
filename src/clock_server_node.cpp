#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "clock_server");
  ros::NodeHandle n;

  ros::Publisher pubClock = n.advertise<rosgraph_msgs::Clock>("/clock",10);
  ros::Rate r (120);
  while (ros::ok()) {
      rosgraph_msgs::Clock msg;
      msg.clock = ros::Time::now();
      pubClock.publish(msg);
      ros::spinOnce();
      r.sleep();
  }

}
