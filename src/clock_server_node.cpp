#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "clock_server");
  ros::NodeHandle n;

  ros::Publisher pubClock = n.advertise<std_msgs::Float64>("/clock",10);
  ros::Rate r (120);
  while (ros::ok()) {
	 std_msgs::Float64 msg;
	 msg.data = ros::Time::now().toSec();
    pubClock.publish(msg);
    ros::spinOnce();
    r.sleep();
  }

}
