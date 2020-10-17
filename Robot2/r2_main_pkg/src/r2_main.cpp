#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <r1_main_pkg/PassDownAction.h>
#include <frontier_pkg_tb/ChoiceMsg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>
#include <boost/bind.hpp>

using namespace std;

class PassDown{
public:
	typedef actionlib::SimpleActionClient<r1_main_pkg::PassDownAction> PassDownClient;
	geometry_msgs::PoseArray pass_down_frontier_send_;
	geometry_msgs::PoseArray pass_down_path_send_;
	vector<uint8_t> chosen_queue_send_;
	int robots_remaining_send_;

	PassDown(geometry_msgs::PoseArray pass_down_frontier_send, geometry_msgs::PoseArray pass_down_path_send, vector<uint8_t> chosen_queue_send, int robots_remaining_send){
		pass_down_frontier_send_ = pass_down_frontier_send;
		pass_down_path_send_ = pass_down_path_send;
		chosen_queue_send_ = chosen_queue_send;
		robots_remaining_send_ = robots_remaining_send;
		SendActionRequest();
	}

	void SendActionRequest(){
		PassDownClient pass_down_r3("R3_main_action_server");
		pass_down_r3.waitForServer();

		r1_main_pkg::PassDownGoal msg;

		msg.pass_down_frontier_req = pass_down_frontier_send_;
		msg.pass_down_path_req = pass_down_path_send_;
		msg.chosen_queue_req = chosen_queue_send_;
		msg.robots_remaining = robots_remaining_send_;

		pass_down_r3.sendGoal(msg, boost::bind(&PassDown::doneCb, this, _1, _2), boost::bind(&PassDown::activeCb, this));
	}

	void doneCb(const actionlib::SimpleClientGoalState& state,
				const r1_main_pkg::PassDownResultConstPtr& result)
	{
		ROS_INFO("[Robot 3 has completed]");
	}

	void activeCb()
	{
		ROS_INFO("[Information has been passed to robot 3]");
	}
};




class MoveBase{
public:
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	geometry_msgs::PoseStamped frontier_pt;
	bool nav_completion;

	MoveBase(geometry_msgs::PoseStamped frontier_pt_){
		ROS_INFO("[MoveBase object constructed]");
		frontier_pt = frontier_pt_;
		SendGoal();
	}

	void SendGoal(){
		MoveBaseClient move_base_r1("tb3_1/move_base", true);
		move_base_r1.waitForServer();

		move_base_msgs::MoveBaseGoal nav_goal;
		nav_goal.target_pose = frontier_pt;
		nav_goal.target_pose.pose.orientation.w = 1.0;

		move_base_r1.sendGoal(nav_goal, boost::bind(&MoveBase::doneCb, this, _1, _2), boost::bind(&MoveBase::activeCb, this), boost::bind(&MoveBase::feedbackCb, this, _1));

		move_base_r1.waitForResult();
	}

	void doneCb(const actionlib::SimpleClientGoalState& state,
				const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		ROS_INFO("[Robot 2 has arrived at its goal]");
		nav_completion = true;
	}

	void activeCb()
	{
		ROS_INFO("[Goal is active]");
	}

	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){

	}
};

class MainAction
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<r1_main_pkg::PassDownAction> as_;
	ros::ServiceClient choice_srv;
	ros::Publisher choice_pub;
	std::string action_name_;

	r1_main_pkg::PassDownResult result_;

public:

	geometry_msgs::PoseArray pass_down_frontier_received;
	geometry_msgs::PoseArray pass_down_path_received;
	vector<uint8_t> chosen_queue_received;
	int robots_remaining_received;

	geometry_msgs::PoseArray pass_down_frontier_send;
	geometry_msgs::PoseArray pass_down_path_send;
	vector<uint8_t> chosen_queue_send;

	geometry_msgs::PoseStamped chosen_pt;
	frontier_pkg_tb::ChoiceMsg choice_msg;

	MainAction(std::string name) :
		as_(nh_, name, boost::bind(&MainAction::executeCB, this, _1), false),
		action_name_(name)
	{
		as_.start();
		choice_srv = nh_.serviceClient<frontier_pkg_tb::ChoiceMsg>("/choose_frontier_r2");
		choice_pub = nh_.advertise<geometry_msgs::PoseStamped>("/r2_chosen_pt", 1000);
		ROS_INFO("[R2 main action server active]");
	}

	~MainAction(void)
	{
	}

	void executeCB(const r1_main_pkg::PassDownGoalConstPtr &msg){
		ros::Rate(1);
		bool success;
		ROS_INFO("[Robot 2 executing decision process]");

		pass_down_frontier_received = msg->pass_down_frontier_req;
		pass_down_path_received = msg->pass_down_path_req;
		chosen_queue_received = msg->chosen_queue_req;
		robots_remaining_received = msg->robots_remaining;

		//Setup choice service
		choice_msg.request.pass_down_frontier_req = pass_down_frontier_received;
		choice_msg.request.pass_down_path_req = pass_down_path_received;
		choice_msg.request.chosen_queue_req = chosen_queue_received;
		choice_msg.request.robots_remaining = robots_remaining_received;

		if (choice_srv.call(choice_msg)){
			ROS_INFO("[Robot 2 sent request for frontier choice]");

			chosen_pt = choice_msg.response.chosen_pt;
			pass_down_frontier_send = choice_msg.response.pass_down_frontier_res;
			pass_down_path_send = choice_msg.response.pass_down_path_res;
			chosen_queue_send = choice_msg.response.chosen_queue_res;

			cout << "size of pass down path: " << pass_down_path_send.poses.size() << endl;
			choice_pub.publish(chosen_pt);

		}

		else{
			ROS_ERROR("[Could not send frontier request for robot 2]");
		}

		// PassDown pass_down(pass_down_frontier_send, pass_down_path_send, chosen_queue_send, robots_remaining_received-1);
		MoveBase move_base(chosen_pt);

		if (as_.isPreemptRequested() || !ros::ok()){
			ROS_INFO("[%s: Preempted]", action_name_.c_str());
			as_.setPreempted();
			success = false;
		}
		else{
			success = true;
		}

		result_.success = success;
		ROS_INFO("[%s: Succeeded]", action_name_.c_str());
		as_.setSucceeded(result_);
	}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "R2_main_node");

	MainAction main_action("R2_main_action_server");
	ros::spin();

	return 0;
}