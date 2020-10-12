/*
 * turtlebot_action_server.h
 *
 *  Created on: May 2, 2018
 *      Author: turtlebot
 */

#ifndef TURTLEBOT_ACTION_SERVER_SRC_TURTLEBOT_ACTION_SERVER_H_
#define TURTLEBOT_ACTION_SERVER_SRC_TURTLEBOT_ACTION_SERVER_H_
#include <ros/ros.h>



#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <string.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include <stdint.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "turtlebot_action_c.hpp"
#include "simple_pid.h"
#include <multi_spin_demo/turtlebot_controlAction.h>
#include "kobuki_msgs/RobotStateEvent.h"



class turtlebot_action_s {
 public:
  turtlebot_action_s(ros::NodeHandle nh, std::string name);
  virtual ~turtlebot_action_s();
  void goalCallback();
  void preemptCallback();
  void executeCallback(const std_msgs::Float64 &msg);
  void waitKobukiCallback(const kobuki_msgs::RobotStateEvent &msg);

  bool isEveryoneConnected();



 protected:
  ros::NodeHandle node;
  ros::NodeHandle nodeSubscriber;
  actionlib::SimpleActionServer<multi_spin_demo::turtlebot_controlAction> as;
  std::string turtleName;
  geometry_msgs::PoseStamped initialGoal;
  geometry_msgs::PoseStamped goal;
  multi_spin_demo::turtlebot_controlActionFeedback feedbackMsg;
  multi_spin_demo::turtlebot_controlActionResult result;
  std::vector<turtlebot_action_c*> turtleClients;
  std::vector<std::uint8_t> connectedTurtles;
  int numberOfTurtle = 0;
  //std::vector<std::vector<std::uint8_t>> serversUp;
  int turtleID;
  std::string launchMode



  std::vector<std::string> robotNames;
  ros::Subscriber subPoseCallback;
  ros::Subscriber subBringUpCallback;
  ros::Publisher pubRobotVelocity;
  bool initKobuki = false;
  std_msgs::Bool spinMsg;
  bool initFlag = true;
  bool success = false;
  pose_controller::SimplePID xPidController;
  pose_controller::SimplePID yPidController;
  geometry_msgs::Twist twistSend;
  double goalYawOrientation =0;
  geometry_msgs::Vector3 robotPose;
  double robotTheta = 0;
  std::string robotName;
  ros::Time time_of_previous_control;
  double dt = 0;
  double ux = 0;
  double uy = 0;
  bool switchMode = false;
};

/* namespace action_server */

#endif /* TURTLEBOT_ACTION_SERVER_SRC_TURTLEBOT_ACTION_SERVER_H_ */
