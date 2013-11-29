/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Fontys Hogescholen Eindhoven.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Fontys Hogescholen Eindhoven nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Kris Piters on 22/11/2013
*********************************************************************/
#include <vector>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector<geometry_msgs::PoseStamped> goals;

bool sendNewGoal;
int  counter = 0;

void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
	ROS_INFO("Goal is complete.");
	sendNewGoal = true;
}

void goalActiveCallback()
{
	ROS_INFO("Goal active.");
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
	
}

void goalsCallback(const geometry_msgs::PoseStamped::ConstPtr& newGoal)
{
	ROS_INFO("New goal added.");
	goals.push_back(*newGoal);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "goal_queue");

	ROS_INFO("Goal Queue Started.");
	
	ros::NodeHandle n;
	
	ros::Subscriber goals_sub = n.subscribe("/goals", 20, goalsCallback);
	
	ros::Rate r(10);
	ROS_INFO("Waiting for first goal to be published...");
	while(goals.size()==0 && ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	ROS_INFO("Got it!");
	
	// Tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	// Wait for the action server to come up
	ROS_INFO("Waiting for the move_base action server to come online...");
	if(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_FATAL("No move_base instance found.");
		ROS_BREAK();
	}
	ROS_INFO("Found move_base.");
	
	sendNewGoal = true;
		
	while(ros::ok())
	{	
		if(sendNewGoal)
		{
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose = goals[counter];		
			goal.target_pose.header.stamp = ros::Time::now();
			
			counter++;
			if(counter >= goals.size()){counter = 0;}

			sendNewGoal = false;
			ac.sendGoal(goal, boost::bind(&goalDoneCallback, _1, _2), boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));
		}
		
		ROS_INFO("Current Goal: %d",(counter+1));
		ROS_INFO("Total Goals:  %d",goals.size());
		ros::spinOnce();
		r.sleep();
	}

	ROS_INFO("Exiting Rosbee Goal Queue...");

	return 0;
}
