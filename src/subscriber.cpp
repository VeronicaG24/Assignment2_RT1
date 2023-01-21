#include <ros/ros.h>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include <cmath>
#include "follow_goal/pos_vel.h"
#include <follow_goal/PlanningActionGoal.h>

using namespace std;


float x_pos, y_pos, x_vel, y_vel, x_goal, y_goal;
float dist_goal, average_vel;
double frequency;


void pos_vel_callback(const follow_goal::pos_vel::ConstPtr& msg) {

	x_pos = msg->x_pos;
	y_pos = msg->y_pos;
	x_vel = msg->x_vel;
	y_vel = msg->y_vel;
	
}

void goal_callback(const follow_goal::PlanningActionGoal::ConstPtr& msg) {	
	
	x_goal = msg->goal.target_pose.pose.position.x;
	y_goal = msg->goal.target_pose.pose.position.y;
	
}

void get_dist_vel_from_goal() {

	dist_goal = sqrt(((x_goal - x_pos)*(x_goal - x_pos)) + ((y_goal - y_pos)*(y_goal - y_pos)));
	average_vel = sqrt((x_vel*x_vel) + (y_vel*y_vel));
	
	
	cout << "Distance from goal: " << dist_goal << "\t\tAverage speed: " << average_vel << "\n";
	
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "subscriber_robot");
	ros::NodeHandle n;
	
	ros::param::get("frequency", frequency);
	ros::Rate rate(frequency);
	
	while(ros::ok()) {
		ros::Subscriber sub1 = n.subscribe("/robot_info", 1, pos_vel_callback);
		
		ros::Subscriber sub2 = n.subscribe("/reaching_goal/goal", 1, goal_callback);
		
		get_dist_vel_from_goal();
		
		rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}
