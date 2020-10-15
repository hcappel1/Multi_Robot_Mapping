#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include <string>



int main(int argc, char **argv){
	ros::init(argc, argv, "test_action_server");
	ros::NodeHandle nh;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>test_action("tb3_0/move_base", true);
	ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 1000);
	geometry_msgs::PoseStamped goal;

	ROS_INFO("waiting for move base to initiate");

	test_action.waitForServer();

	move_base_msgs::MoveBaseGoal nav_goal;
	nav_goal.target_pose.pose.position.x = 11.92;
	nav_goal.target_pose.pose.position.y = 14.775;
	nav_goal.target_pose.pose.orientation.w = 1.0;
	nav_goal.target_pose.header.stamp = ros::Time::now();
	nav_goal.target_pose.header.frame_id = "map";
	goal.pose.position.x = 11.92;
	goal.pose.position.y = 14.775;
	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "map";
	ros::Duration(1.5);
	goal_pub.publish(goal);


	ROS_INFO("move base started sending goal...");

	test_action.sendGoal(nav_goal);


	test_action.waitForResult();

	actionlib::SimpleClientGoalState state = test_action.getState();
	ROS_INFO("Current state: %s", state.toString().c_str());

	// actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction>test_action_fib("fibonacci", true);
	// test_action_fib.waitForServer();
	// actionlib_tutorials::FibonacciGoal goal;
	// goal.order = 20;
	// test_action_fib.sendGoal(goal);

	// test_action_fib.waitForResult(ros::Duration(10.0));

	ros::spin();
	return 0;
}