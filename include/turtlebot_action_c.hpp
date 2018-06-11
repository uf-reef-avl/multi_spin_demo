/*
 * turtlebot_action_c.h
 *
 *  Created on: May 3, 2018
 *      Author: turtlebot
 */

#ifndef TURTLEBOT_ACTION_SERVER_SRC_TURTLEBOT_ACTION_C_HPP_
#define TURTLEBOT_ACTION_SERVER_SRC_TURTLEBOT_ACTION_C_HPP_
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Bool.h"
#include <actionlib/client/simple_action_client.h>
#include <string.h>
#include "multi_spin_demo/turtlebot_controlAction.h"






class turtlebot_action_c {
 public:
  turtlebot_action_c(ros::NodeHandle nh, std::string name);
  virtual ~turtlebot_action_c();
  void sendOrder(std_msgs::Bool spin_goal);
  void doneCallback(const actionlib::SimpleClientGoalState& state, const multi_spin_demo::turtlebot_controlResultConstPtr& result);
  void activeCallback();
  void feedbackCallback(const multi_spin_demo::turtlebot_controlFeedbackConstPtr & feedback);
  std::string getTurtleName() const;
  multi_spin_demo::turtlebot_controlFeedbackConstPtr getLastFeedback() const;
  bool isClientConnected() const;


 protected:
  actionlib::SimpleActionClient<multi_spin_demo::turtlebot_controlAction> ac;
  geometry_msgs::PoseStamped currentPose;
  std::string turtleName;
  //int numberOfMachine;
  //std::vector<bool> serverConnexions;
  multi_spin_demo::turtlebot_controlFeedbackConstPtr lastFeedback = NULL;


};
#endif /* TURTLEBOT_ACTION_SERVER_SRC_TURTLEBOT_ACTION_C_HPP_ */
