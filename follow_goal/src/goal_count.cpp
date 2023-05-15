/**
*\file goal_count.cpp
*\brief ROS node for counting goals reached and deleted
*\author Veronica Gavagna
*\version 0.1
*\date 26/01/2023
**/

#include <ros/ros.h>
#include <follow_goal/GoalNumber.h>
#include "assignment_2_2022/PlanningActionResult.h"

using namespace std;

int status; ///< global varibles for status
int reached_goal=0; ///< global counter for reached goals
int deleted_goal=0; ///< global counter for deleted goals


/**
*\brief goal service callback
*\param req, res
*\return true

* This function get the number of reached and deleted goals
* and send information on custom service result
**/
bool goal_service_callback(follow_goal::GoalNumber::Request &req, follow_goal::GoalNumber::Response &res) {
	res.reached = reached_goal;
	res.deleted = deleted_goal;
	return true;
}


/**
*\brief Counts goals reached and deleted
*\param msg
*\return void

* This function count the number of reached and deleted goals
* based on the status of the current goal. <BR>
* If the statuse is succeeded, then the number of reached goal is incremented,
* otherwise, if the status is preemptive, the number of deleted goal is incremented.
**/
void count_reach_delete_callback(const assignment_2_2022::PlanningActionResult::ConstPtr& msg) {
	
	status=msg->status.status;
	ROS_INFO("\nStatus goal:%d", status);
	
	// check if the current goal is succeeded
	if (status==3) 
		reached_goal++;
	
	// check if the current goal is preemptive
	else if (status==2) 
		deleted_goal++;
}


/**
*\brief Main function
*\param argc, argv
*\return 0

* This function manages the goal_count node. <BR>
* It does the ROS init, set the NodeHandle to access the comunication with the ROS system, 
* creates a subscriber to "/reaching_goal/result" and a service server to "/result". <BR>
**/
int main (int argc, char** argv) {
	
	//init
	ros::init(argc, argv, "goal_count");
	
	// NodeHandle is the main access point to communications with the ROS system
	ros::NodeHandle n;
	
	// Subscriber
	ros::Subscriber sub = n.subscribe("/reaching_goal/result", 1, count_reach_delete_callback);
	
	// Service
	ros::ServiceServer service = n.advertiseService("/result", goal_service_callback);
	
	sleep(1);
	ros::spin();
	return 0;
	
}

