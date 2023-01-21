#include <ros/ros.h>
#include <follow_goal/GoalNumber.h>
#include "assignment_2_2022/PlanningActionResult.h"

using namespace std;

int status;
int reached_goal=0, delelated_goal=0;

/* similar to callback function, set in the custom service:
	number of reach goals;
	number of eliminated goals */
bool goal_service(follow_goal::GoalNumber::Request &req, follow_goal::GoalNumber::Response &res) {
	res.reached=reached_goal;
	res.deleted=delelated_goal;
	return true;
}

/* this callback takes the status of current node and counts goals*/
void count_reach_delete_callback(const assignment_2_2022::PlanningActionResult::ConstPtr& msg) {
	
	status=msg->status.status;
	ROS_INFO("\nStatus goal:%d", status);
	
	// check if the current goal is succeeded
	if (status==3) 
		reached_goal++;
	
	// check if the current goal is preemptive
	else if (status==2) 
		delelated_goal++;
}

int main (int argc, char** argv) {
	
	/* you must call one of the versions of ros::init() before using any other
  	part of the ROS system. */
	ros::init(argc, argv, "goal_count");
	
	// NodeHandle is the main access point to communications with the ROS system.
	ros::NodeHandle n;
	
	// define a subscriber, which takes information from topi
	ros::Subscriber sub = n.subscribe("/reaching_goal/result", 1, count_reach_delete_callback);
	
	// define a service, which send information on custom service result
	ros::ServiceServer service=n.advertiseService("/result", goal_service);
	sleep(1);
	ros::spin();
	return 0;
	
}

