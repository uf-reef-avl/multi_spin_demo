/*
 * turtlebot_action_server.cpp
 *
 *  Created on: May 2, 2018
 *      Author: turtlebot
 */

#include "turtlebot_action_s.hpp"





turtlebot_action_s::turtlebot_action_s(ros::NodeHandle nh, std::string name):node(nh),as(nh,name,false), turtleName(name){

 //call the goal callback everytime a new goal is received
  as.registerGoalCallback(boost::bind(&turtlebot_action_s::goalCallback,this));
  //call the preemption callback everytime a cancel request is received
  as.registerPreemptCallback(boost::bind(&turtlebot_action_s::preemptCallback,this));
  // subscribe to the optitrack node
  subPoseCallback = node.subscribe("/clock", 10 , &turtlebot_action_s::executeCallback, this);
  subBringUpCallback = node.subscribe("/"+turtleName+"/mobile_base/events/robot_state", 10 , &turtlebot_action_s::waitKobukiCallback, this);
  //advertise the kokubi_turtlebot node
  pubRobotVelocity = node.advertise<geometry_msgs::Twist>("/"+turtleName+"/mobile_base/commands/velocity",10);




  //start the turtlebot server node
  as.start();
  ROS_INFO("%s : server started", turtleName.c_str());

  std::vector<std::string> robotListNames;
  if(node.getParam("turtleList/names",robotListNames))
  {
    robotNames = robotListNames; // find all the names of the robot
    ROS_INFO("%s has found the others robot's names",turtleName.c_str());
  }
  else
  {
    ROS_ERROR("%s hasn't found the other robot's names",turtleName.c_str());
  }



  // compute the number of turtlebot
  numberOfTurtle = robotNames.size();
  // create a int vector in order to check the connection between the turtle
  connectedTurtles.resize(numberOfTurtle);

  // create the feedback message that will send the vector to the others turtles
  std::string feedback_label = "connexion vector of"+turtleName;
  feedbackMsg.feedback.servers_up.layout.dim.push_back(std_msgs::MultiArrayDimension());
  feedbackMsg.feedback.servers_up.layout.dim[0].label = feedback_label;
  feedbackMsg.feedback.servers_up.layout.dim[0].size = numberOfTurtle;
  feedbackMsg.feedback.servers_up.layout.dim[0].stride = 1;



  int index = 0;
  for(auto it = robotNames.begin(); it != robotNames.end(); it++) //initialize list of clients
   {
      std::string currentName = *it;
      if(currentName != turtleName){ // create a client for each others turtles in the simulation
        turtleClients.push_back(new turtlebot_action_c(nh,currentName));

      }
      else  // retrieve parameters for this server
      {
    	//register the turtle ID
        turtleID = index;
        // add an empty client for the turtle ID in the client vector
        turtleClients.push_back(NULL);
        //initialize the launchmode of turtle
        std::string tempLaunchMode;
          if(node.getParam("turtleList/launchMode/"+currentName,tempLaunchMode))
          {
	      launchMode = tempLaunchMode;
              ROS_INFO("initial the launch mode of %s set \n",currentName.c_str());
          }
          else
          {
              ROS_ERROR("failed to set launch mode %s \n",currentName.c_str());
          }

        //initialize the initial goal parameter of the turtle
        std::vector<double> tempInitialGoal;
        if(node.getParam("turtleList/initialGoals/"+currentName,tempInitialGoal))
               {
                 initialGoal.pose.position.x= tempInitialGoal[0];
                 initialGoal.pose.position.y= tempInitialGoal[1];
                 initialGoal.pose.position.z= tempInitialGoal[2];
                  ROS_INFO("initial goal of %s set \n",currentName.c_str());
               }
               else
               {
                 ROS_ERROR("failed to set initial goal for %s \n",currentName.c_str());
               }
        //initialize the XPIDThetGains parameter of the turtle
        std::vector<double> tempXPIDThetGains;
        if(node.getParam("turtleList/xPIDThet/"+currentName,tempXPIDThetGains))
               {
                  xPidController.setGains(tempXPIDThetGains[0],tempXPIDThetGains[1],tempXPIDThetGains[2],tempXPIDThetGains[3]);
                  ROS_INFO("x gains of %s set \n",currentName.c_str());
               }
               else
               {
                 ROS_ERROR("failed to set x gains for %s \n",currentName.c_str());
               }
        //initialize the YPIDThetGains parameter of the turtle
        std::vector<double> tempYPIDThetGains;
        if(node.getParam("turtleList/yPIDThet/"+currentName,tempYPIDThetGains))
               {
                  yPidController.setGains(tempYPIDThetGains[0],tempYPIDThetGains[1],tempYPIDThetGains[2],tempYPIDThetGains[3]);
                  ROS_INFO("y gains of %s set \n",currentName.c_str());
               }
               else
               {
                 ROS_ERROR("failed to set y gains for %s \n",currentName.c_str());
               }

        //initialize the XMinMax parameter of the turtle
        std::vector<double> xMinMax;
        if(node.getParam("turtleList/xMinMax/"+currentName,xMinMax))
        {
          xPidController.setMinMax(xMinMax[0],xMinMax[1]);
          ROS_INFO("xMinMax setup for %s \n",currentName.c_str());
        }
        else
        {
          ROS_ERROR("Failed to set %s xMinMax",currentName.c_str() );
        }
        //initialize the YMinMax parameter of the turtle
        std::vector<double> yMinMax;
        if(node.getParam("turtleList/yMinMax/"+currentName,yMinMax))
        {
          yPidController.setMinMax(yMinMax[0],yMinMax[1]);
          ROS_INFO("yMinMax setup for %s \n",currentName.c_str());
        }
        else
        {
          ROS_ERROR("Failed to set %s yMinMax",currentName.c_str() );
        }
      }
      // increase the turtle possible ID every loop
      index += 1;
   }


  int indexNextServer;
  // the clients are linked to the next turtlebot server
  if(turtleID == numberOfTurtle-1) // if the turtle ID is the last one, the client is bound to the fist turtle
  {
	  indexNextServer = 0;

  }
  else
  {
	  indexNextServer = turtleID + 1;
  }


  ros::Rate loopRate(120);

  spinMsg.data = true;
  //every linked client send one goal to the next turtlebot to receive his feedback
  turtleClients[indexNextServer]->sendOrder(spinMsg);

  ROS_INFO("initialisation of the parameters of  %s is finished",turtleName.c_str());


  //rate of the refreshing loop

  while(initFlag)
  {

	  //clear the connection's vector
	  connectedTurtles.clear();

	  //check the connection data vector
	  for(int i = 0; i < numberOfTurtle ; i++)
		{
		  	  //retrieve the vector of the next turtlebot
			  if(turtleClients[indexNextServer]->getLastFeedback() != NULL)
			  {

				  connectedTurtles[i] = turtleClients[indexNextServer]->getLastFeedback()->servers_up.data[i];

			  }
			  else
			  {
				  // if the current turtlebot can't just add 0 to all the connection
				  connectedTurtles[i] = 0;

			  }
		  }
	  //if the client to the next turtlebot of the current turtlebot is connected to the next one, add a 1 in the connection vector
	  if(turtleClients[indexNextServer]->isClientConnected() and (launchMode == "gazebo" or (launchMode == "turtlebot" and initKobuki == true)))
	 {
	 	connectedTurtles[turtleID] = 1;

	 }
	 else
	 {//if the client to the next turtlebot of the current turtlebot is not connected to the next one, add a 0 in the connection vector
	 	 connectedTurtles[turtleID] = 0;
	 }
	  //clear the previous feedback sended
	  feedbackMsg.feedback.servers_up.data.clear();
	  for(int i = 0; i < numberOfTurtle ; i++)
		  {
		  // add the newly created
		  feedbackMsg.feedback.servers_up.data.push_back(connectedTurtles[i]);

		  }
	 // if the shared vector is [1.1...1.1] finish the loop
    initFlag = !isEveryoneConnected();


    //send the vector to the previous turtlebot if the server as already receive some order from client (switchmode is initialized to false)
    if(switchMode == true)
    { 
	as.publishFeedback(feedbackMsg.feedback);
    }
   
  
    
    ros::spinOnce();
    loopRate.sleep();
  }
  ROS_INFO("initialisation of --------------- %s --------------- finished",turtleName.c_str());

  // we can now execute orders because every server/client is setup
  goal.pose.position.x = initialGoal.pose.position.x;
  goal.pose.position.y = initialGoal.pose.position.y;

  srand(turtleID);


}

turtlebot_action_s::~turtlebot_action_s() {
  // delete the client' s vector
  for(auto it = turtleClients.begin(); it != turtleClients.end(); it++)
  {
    delete *it;
  }
}


bool turtlebot_action_s::isEveryoneConnected()
//check if all the slot of the shared vector is set to 1
{

  bool everyoneConnected = true;
  for(int i = 0; i < numberOfTurtle; i++)
  {

	  everyoneConnected = everyoneConnected && connectedTurtles[i];
  }
  return everyoneConnected;
}

void turtlebot_action_s::goalCallback()
{
//set the new goal of the turtlebot send by others turtlebot
  ROS_INFO("new goal received, switch mode %s",turtleName.c_str());
  const std_msgs::Bool spinTemp = as.acceptNewGoal()->temp_goal;
  switchMode = spinTemp.data;
  /*goal.pose.position.x = goal_temp.pose.position.x;
  goal.pose.position.y = goal_temp.pose.position.y;
  goal.pose.position.z = goal_temp.pose.position.z;
  goal.pose.orientation.z = tf::getYaw(goal_temp.pose.orientation);*/

 /* ROS_INFO("goal set");
  ROS_INFO("goal set %f",goal.pose.position.x);
  ROS_INFO("goal set %f",goal.pose.position.y);
  ROS_INFO("goal set %f",goal.pose.position.z);*/


}
void turtlebot_action_s::preemptCallback()
//if the server receive a cancel request, do things
{
  //ROS_INFO("%s server Preempted", turtleName.c_str());


}
void turtlebot_action_s::executeCallback(const std_msgs::Float64 &msg)
{
	// everytime optitrack send the turtlebot position = apply a velocity

// check if the initialisation is finished and the server still active
  if(!as.isActive() || initFlag)
    return;
// check is the server is preempted or ros is still running
  if (!as.isPreemptRequested() || ros::ok())
       {
	//compute the velocity to get to to the goal position
	/*ROS_INFO("transform translation %f",msg.transform.translation.x);
	ROS_INFO("transform translation %f",msg.transform.translation.y);
    dt = (msg.header.stamp - time_of_previous_control).toSec();
    time_of_previous_control = msg.header.stamp;
    ux = xPidController.computePID(goal.pose.position.x,msg.transform.translation.x,dt);
    uy = yPidController.computePID(goal.pose.position.y,msg.transform.translation.y,dt);

    twistSend.linear.x = sqrt(pow(ux, 2) + pow(uy, 2));
    twistSend.angular.z = (atan2(uy, ux))-tf::getYaw(msg.transform.rotation);
    std::cout<< " Error Theta " << twistSend.angular.z * 180/3.14 << std::endl;

    if(twistSend.linear.x > 0.5)
    {
    	twistSend.linear.x = 0.5;
    }
    if(twistSend.linear.x < -0.5)
    {
    	twistSend.linear.x = -0.5;
    }
    if(twistSend.angular.z > 0.5)
    {
    	twistSend.angular.z = 0.5;
    }
    if(twistSend.angular.z < -0.5)
    {
    	twistSend.angular.z = -0.5;
    }*/

// advertise the kokuby node to apply the velocity


	  int randomNumber = rand() % 10000;
	  if(randomNumber <= 3)
	  {
		  ROS_INFO("random number %i",randomNumber);
		  ROS_INFO("switch from %s",turtleName.c_str());
		  	switchMode = !switchMode;
			for(int i = 0; i < numberOfTurtle ; i++)
					{
				if(i != turtleID)
				{
					spinMsg.data = switchMode;
					turtleClients[i]->sendOrder(spinMsg);
				}
					}
	  }

	  if(switchMode)
	  {
		  twistSend.angular.z = 0.7;
	  }
	  else
	  {
		  twistSend.angular.z = -0.7;
	  }


	  pubRobotVelocity.publish(twistSend);
 //send a feedback to the previous turtlebot
    /*feedbackMsg.feedback.current_position.pose.position.x = msg.transform.translation.x;
    feedbackMsg.feedback.current_position.pose.position.y = msg.transform.translation.y;
    feedbackMsg.feedback.current_position.pose.position.z = msg.transform.translation.z;
    feedbackMsg.feedback.current_position.pose.orientation.w = msg.transform.rotation.w;
    feedbackMsg.feedback.current_position.pose.orientation.x = msg.transform.rotation.x;
    feedbackMsg.feedback.current_position.pose.orientation.y = msg.transform.rotation.y;
    feedbackMsg.feedback.current_position.pose.orientation.z = msg.transform.rotation.z;*/
    feedbackMsg.feedback.x_velocity = ux;
    feedbackMsg.feedback.y_velocity = uy;


    as.publishFeedback(feedbackMsg.feedback);
       }

//if the position and the goal position is the same then send the result to the previous turtlebot
  if(success)
  {
    ROS_INFO("%s: Succeeded", turtleName.c_str());
    as.setSucceeded(result.result);
  }

}

void turtlebot_action_s::waitKobukiCallback(const kobuki_msgs::RobotStateEvent &msg)
{

	if (msg.state == 1)
	{
		initKobuki = true;
	}
}


