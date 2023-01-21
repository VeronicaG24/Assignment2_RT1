#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <sstream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <follow_goal/PlanningAction.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

using namespace std;

float x_pos, y_pos;
bool ask = true;
char answer;

void ask_goal() {
	cout<<"Set the x of the goal:\n";
	cin>>x_pos;
	cout<<"Set the y of the goal:\n";
	cin>>y_pos;
}

int main (int argc, char **argv) {

	char ans = 'n';	
	ros::init(argc, argv, "goal_set");
	  
	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<follow_goal::PlanningAction> ac("/reaching_goal", true);
	  
	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time
	  
	while(ros::ok()) {
		actionlib::SimpleClientGoalState state = ac.getState();
		
		if (ask == true) {
			//ask the user to set a  goal
			ask_goal();
			ROS_INFO("Action server started, sending goal.");
			// send a goal to the action
			follow_goal::PlanningGoal goal;
			goal.target_pose.pose.position.x = x_pos;
			goal.target_pose.pose.position.y = y_pos;
			ac.sendGoal(goal);
			answer = 'm';
			cout << "Do you want to cancel the goal? [y/n]: ";
		}
		
		if (state.toString().compare("SUCCEEDED")) {
			
			while(answer != 'y' && answer != 'n' && state.toString().compare("SUCCEEDED")) {
				
				//sleep(2);
				answer = getchar();
				
				/*if (tolower(answer) == 'm') {
					cout << "entered: " << answer << endl;
					sleep(1);
				}*/
				
				if(answer == 'y' || answer == 'Y') {
					ac.cancelGoal();
					sleep(1);
					ask = true;
				}
				else if (answer == 'n' || answer == 'N') {
					ask = false;
				}
				/*else {
					answer = 'm';
					ask = false;
				}*/
			}
		}
		else {
			cout << "ask true";
			ask = true;
		}
	}
	  
	return 0;
}



