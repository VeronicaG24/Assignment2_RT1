/**
*\file goal_set.cpp
*\brief ROS node for managing the menu
*\author Veronica Gavagna
*\version 0.1
*\date 26/01/2023
**/

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "follow_goal/GoalNumber.h"
#include <unistd.h>
#include <string>
#include <actionlib/client/terminal_state.h>
#include <follow_goal/PlanningAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

int reached, deleted; ///< global varibles for reached and deleted goals
float x_pos, y_pos; ///< global varibles for (x,y) coordinates position of the robot

ros::ServiceClient client; ///< instance client
follow_goal::GoalNumber srv; ///< instance custome service


/**
*\brief get_number_goals function
*\param 
*\return void

* This function calls the custom service, get the number of goals reached 
* and deleted and shows the results on the terminal.
**/
void get_number_goals() {

	client.waitForExistence();
    	client.call(srv);
    	
    	reached = srv.response.reached;
    	
    	deleted=srv.response.deleted;
    	
    	cout<<"\nNumber of reached goals: "<<reached<<"\nNumber of delated goals: "<<deleted<<"\n";
}


/**
*\brief input_menu function
*\param 
*\return answ

* This function shows the menu and get thge answer from the user to the number corresponding to the action wanted. <BR>
**/
int input_menu() {
	char answ;

	cout<<"\n\n--------------------------------------------------------\nSelect an option:\n  1) Set goal coordinates\n  2) Delete current goal\n  3) Number of deleted and reached goal\n  4) Exit\nPress the number corresponding to the choosen option\n--------------------------------------------------------\n\n";
	
  	cin>>answ;
  	
  	return answ;
}


/**
*\brief ask_goal function
*\param 
*\return void

* This function ask the user to set the (x,y) corrdinates for setting of the new goal.
**/
void ask_goal() {
	cout<<"Set the x of the goal:\n";
	cin>>x_pos;
	cout<<"Set the y of the goal:\n";
	cin>>y_pos;
	cout<<"Goal set!\n";
}


/**
*\brief Main function
*\param argc, argv
*\return 0

* This function manages the goal_set node. <BR>
* It does the ROS init, set the NodeHandle to access the comunication with the ROS system, 
* and creates an action client to "/reaching_goal". <BR>
* It manages the option menu where the user can set the goal coordinates, delate the current goal set,
* get the number of reached and deleted goals, and exit the menu.
**/
int main(int argc, char **argv) {

  	char answer; // local variable for answer choosed by the user

  	int succeed; // local variable to state if the goal is reached

  	int set_goal = 0; // local variable to state if the goal is set
  	
  	//init
	ros::init(argc, argv, "goal_set");
	// NodeHandle is the main access point to communications with the ROS system.
	ros::NodeHandle n;
	// Action client creation
	actionlib::SimpleActionClient<follow_goal::PlanningAction> ac("/reaching_goal", true);
	// get the state from the action client
	actionlib::SimpleClientGoalState state = ac.getState();

  	// get state from action client
  	follow_goal::PlanningGoal goal;
  	ROS_INFO("Waiting for action server to start.");
  	
  	/* wait for the action server to start; 
  	it will wait for infinite time*/
  	ac.waitForServer();
	
	while (ros::ok()) {
  		answer = input_menu();
  		state = ac.getState();
  		succeed = state.toString().compare("SUCCEEDED");
  		
  		if (succeed == 0 && set_goal)
  			set_goal--;
  		
  		switch(answer) {
	  		case '1': //Set goal coordinates
	  			state = ac.getState();
	  			succeed = state.toString().compare("SUCCEEDED");
	  			if (set_goal && succeed != 0) {
	  				cout<<"You have to delete a goal before setting a new one\n";
	  			}
	  				
	  			if (succeed == 0 && set_goal) {
	  				set_goal--;
	  			}
	  			else if (!set_goal) {
	  				ask_goal();
					goal.target_pose.pose.position.x=x_pos;
					goal.target_pose.pose.position.y=y_pos;
					ac.sendGoal(goal);
					set_goal++;
				}
				sleep(1);
				break;
				
			case '2': // Delete current goal
				state = ac.getState();
	  			succeed = state.toString().compare("SUCCEEDED");
				if (set_goal && succeed != 0) {
					ac.cancelGoal();
					cout<<"Goal deleted\n";
					set_goal--;
				}
				else {
					cout<<"\nNo goal to delete! Set a new one, or exit\n";
				}
				sleep(1);
				break;
			
	    		case '3': // Number of deleted and reached goal
	    			client = n.serviceClient<follow_goal::GoalNumber>("/result");
	    			get_number_goals();
	    			sleep(1);
	    			break;
	    			
	    		case '4': // Exit
	    			if (set_goal) {
	    				ac.cancelGoal();
	    			}
	    			exit(0);
	    			break;
	    		
			default:
				cout<<"\nMenu:\nChoose a number\n";
				sleep(1);
		}
			
	}
  	return 0;
}
