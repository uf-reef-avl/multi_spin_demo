/*
 * turtlebot_action_c.cpp
 *
 *  Created on: May 3, 2018
 *      Author: turtlebot
 */

#include "turtlebot_action_c.hpp"

turtlebot_action_c::turtlebot_action_c(ros::NodeHandle nh, std::string name) : ac(name,true), turtleName(name)
{
  ROS_INFO("Waiting for action server to start");

  ac.waitForServer();
  ROS_INFO("%s client started", name.c_str());
  if( ac.isServerConnected() )
  {
      ROS_INFO( "--------------%s Connected!", name.c_str() );
  }
  else
  {
      ROS_INFO("---------------%s Not connected", name.c_str());
  }


}


turtlebot_action_c::~turtlebot_action_c() {
  // TODO Auto-generated destructor stub
}



std::string turtlebot_action_c::getTurtleName() const
{
  return turtleName;
}
multi_spin_demo::turtlebot_controlFeedbackConstPtr turtlebot_action_c::getLastFeedback() const
{
  return lastFeedback;
}

bool turtlebot_action_c::isClientConnected() const
{
  return ac.isServerConnected();
}


void turtlebot_action_c::sendOrder(std_msgs::Bool spin_goal){

	multi_spin_demo::turtlebot_controlGoal goal;
  goal.temp_goal = spin_goal;

ac.sendGoal(
    goal,
    boost::bind(&turtlebot_action_c::doneCallback, this, _1, _2),
    boost::bind(&turtlebot_action_c::activeCallback, this),
    boost::bind(&turtlebot_action_c::feedbackCallback, this, _1)
	);
}





void turtlebot_action_c::doneCallback(const actionlib::SimpleClientGoalState& state, const multi_spin_demo::turtlebot_controlResultConstPtr& result){

}

void turtlebot_action_c::activeCallback(){

}

void turtlebot_action_c::feedbackCallback(const multi_spin_demo::turtlebot_controlFeedbackConstPtr & feedback){
    lastFeedback = feedback;

}






