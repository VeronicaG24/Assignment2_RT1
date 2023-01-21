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

// global variables
int reached, deleted;
float x_pos, y_pos;

// istance client and custome service
ros::ServiceClient client;
follow_goal::GoalNumber srv;

// this function calls the custome service, and takes the number of goals
void get_number_goals() {
	client.waitForExistence();
    	client.call(srv);
    	
    	// peek reached goals
    	reached=srv.response.reached;
    	
    	// peek eliminate goals
    	deleted=srv.response.deleted;
    	cout<<"\nNumber of reached goals: "<<reached<<"\nNumber of delated goals: "<<deleted<<"\n";
}

// this function takes and return the user input
int input_menu(char answer) {
	cout<<"\n\n--------------------------------------------------------\nSelect an option:\n  1) Set goal coordinates\n  2) Delete current goal\n  3) Number of deleted or reached goal\n  4) Exit\nPress the number corresponding to the choosen option\n--------------------------------------------------------\n\n";
  	cin>>answer;
  	return answer;
}

void ask_goal() {
	cout<<"Set the x of the goal:\n";
	cin>>x_pos;
	cout<<"Set the y of the goal:\n";
	cin>>y_pos;
	cout<<"Goal set!\n";
}


int main(int argc, char **argv) {
	/* you must call one of the versions of ros::init() before using any other
  	part of the ROS system. */
	ros::init(argc, argv, "goal_set");
	
	// NodeHandle is the main access point to communications with the ROS system.
	ros::NodeHandle n;
	
	// create the action client
	actionlib::SimpleActionClient<follow_goal::PlanningAction> ac("/reaching_goal", true);
	
	// take the state from the action client
	actionlib::SimpleClientGoalState state = ac.getState();
  	
  	char answer;
  	int res;
  	// variable stands for if a goal is set
  	int set_goal = 0;
  	follow_goal::PlanningGoal goal;
  	ROS_INFO("Waiting for action server to start.");
  	
  	/* wait for the action server to start; 
  	it will wait for infinite time*/
  	ac.waitForServer();
	
	while (ros::ok()) {
  		answer = input_menu(answer);
  		state = ac.getState();
  		res = state.toString().compare("SUCCEEDED");
  		if (res == 0 && set_goal)
  			set_goal--;
  		
  		switch(answer) {
  		case '1':
  			state = ac.getState();
  			res = state.toString().compare("SUCCEEDED");
  			if (set_goal && res != 0) {
  				cout<<"You have to delete a goal before setting a new one\n";
  			}
  				
  			if (res == 0 && set_goal) {
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
		case '2':
			state = ac.getState();
  			res = state.toString().compare("SUCCEEDED");
			if (set_goal && res != 0) {
				ac.cancelGoal();
				cout<<"Goal deleted\n";
				set_goal--;
			}
			else {
				cout<<"\nNo goal to delete! Set a new one, or exit\n";
			}
			sleep(1);
			break;
		
    		case '3':
    			client = n.serviceClient<follow_goal::GoalNumber>("/result");
    			get_number_goals();
    			sleep(1);
    			break;
    			
    		case '4':
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
