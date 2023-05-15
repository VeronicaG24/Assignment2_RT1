/**
*\file subscriber.cpp
*\brief ROS node for computing the distance from the goal and average velocity
*\author Veronica Gavagna
*\version 0.1
*\date 26/01/2023
**/

#include <ros/ros.h>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include <cmath>
#include "follow_goal/pos_vel.h"
#include <follow_goal/PlanningActionGoal.h>

using namespace std;

float x_pos, y_pos; ///< global varibles for (x,y) positions of the robot
float x_vel, y_vel; ///< global varibles for (x,y) velocities of the robot
float x_goal, y_goal; ///< global varibles for (x,y) positions of the goal

float dist_goal; ///< global varible for the distance from the goal
float average_vel; ///< global varible for the average speed of the robot

double frequency; ///< global varibles for the frequency rate

ros::Subscriber sub1; ///< instance for the first subscriber
ros::Subscriber sub2; ///< instance for the second subscriber


/**
*\brief pos_vel_callback function
*\param msg
*\return void

* This function get position and velocity of the robot from custom topic pos_vel.
**/
void pos_vel_callback(const follow_goal::pos_vel::ConstPtr& msg) {

	x_pos = msg->x_pos;
	y_pos = msg->y_pos;
	x_vel = msg->x_vel;
	y_vel = msg->y_vel;
	
}


/**
*\brief goal_callback function
*\param msg
*\return void

* This function gets coordinates of the goal.
**/
void goal_callback(const follow_goal::PlanningActionGoal::ConstPtr& msg) {	
	
	x_goal = msg->goal.target_pose.pose.position.x;
	y_goal = msg->goal.target_pose.pose.position.y;
	
}


/**
*\brief get_dist_vel_from_goal function
*\param 
*\return void

* This function computes distance from the goal and average speed of the robot, and prints the results.
**/
void get_dist_vel_from_goal() {

	dist_goal = sqrt(((x_goal - x_pos)*(x_goal - x_pos)) + ((y_goal - y_pos)*(y_goal - y_pos)));
	average_vel = sqrt((x_vel*x_vel) + (y_vel*y_vel));
	
	
	cout << "Distance from goal: " << dist_goal << "\t\tAverage speed: " << average_vel << "\n";
	
}


/**
*\brief Main function
*\param argc, argv
*\return 0

* This function manages the subscriber node. <BR>
* It does the ROS init, set the NodeHandle to access the comunication with the ROS system, 
* set the rate frequency, creates a subscriber to "/robot_info" and another one to "/reaching_goal/goal". <BR>
* It also calls the function to compute the distance from the goal and the average speed.
**/
int main(int argc, char **argv) {
	
	//init
	ros::init(argc, argv, "subscriber_robot");
	
	//NodeHandle for main access point to communications with ROS system
	ros::NodeHandle n;
	
	// set rate frequency
	ros::param::get("frequency", frequency);
	ros::Rate rate(frequency);
	
	while(ros::ok()) {
		// Subscriber for position and velocity
		sub1 = n.subscribe("/robot_info", 1, pos_vel_callback);
		
		// Subscriber for goal coordinates
		sub2 = n.subscribe("/reaching_goal/goal", 1, goal_callback);
		
		// calculation of distance from the goal and average speed
		get_dist_vel_from_goal();
		
		rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}
