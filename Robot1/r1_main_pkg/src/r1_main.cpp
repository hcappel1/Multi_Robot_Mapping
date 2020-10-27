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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<r1_main_pkg::PassDownAction> PassDownClient;

class PassDown{
public:

	PassDownClient pass_down_r2;
	geometry_msgs::PoseArray pass_down_frontier_send_;
	geometry_msgs::PoseArray pass_down_path_send_;
	vector<uint8_t> chosen_queue_send_;
	int robots_remaining_send_;
	bool backup_send_;

	PassDown(geometry_msgs::PoseArray pass_down_frontier_send, geometry_msgs::PoseArray pass_down_path_send, vector<uint8_t> chosen_queue_send, int robots_remaining_send, bool backup_send) : pass_down_r2("R2_main_action_server", true)
	{
		pass_down_frontier_send_ = pass_down_frontier_send;
		pass_down_path_send_ = pass_down_path_send;
		chosen_queue_send_ = chosen_queue_send;
		robots_remaining_send_ = robots_remaining_send;
		backup_send_;
		SendActionRequest();
	}

	void SendActionRequest(){
		pass_down_r2.waitForServer();

		r1_main_pkg::PassDownGoal msg;

		msg.pass_down_frontier_req = pass_down_frontier_send_;
		msg.pass_down_path_req = pass_down_path_send_;
		msg.chosen_queue_req = chosen_queue_send_;
		msg.robots_remaining = robots_remaining_send_;
		msg.backup = backup_send_;

		pass_down_r2.sendGoal(msg, boost::bind(&PassDown::doneCb, this, _1, _2), boost::bind(&PassDown::activeCb, this));
	}

	void doneCb(const actionlib::SimpleClientGoalState& state,
				const r1_main_pkg::PassDownResultConstPtr& result)
	{
		ROS_INFO("[Robot 2 has completed]");
	}

	void activeCb()
	{
		ROS_INFO("[Information has been passed to robot 2]");
	}
};




class MoveBase{
public:

	MoveBaseClient move_base_r1;
	geometry_msgs::PoseStamped frontier_pt;
	bool nav_completion;

	MoveBase(geometry_msgs::PoseStamped frontier_pt_) : move_base_r1("tb3_0/move_base", true)
	{
		ROS_INFO("[MoveBase object constructed]");
		MoveBaseClient move_base_r1("tb3_0/move_base", true);
		frontier_pt = frontier_pt_;
		SendGoal();
	}

	void SendGoal(){
		move_base_r1.waitForServer();

		move_base_msgs::MoveBaseGoal nav_goal;
		nav_goal.target_pose = frontier_pt;
		nav_goal.target_pose.pose.orientation.w = 1.0;

		move_base_r1.sendGoal(nav_goal, boost::bind(&MoveBase::doneCb, this, _1, _2), boost::bind(&MoveBase::activeCb, this), boost::bind(&MoveBase::feedbackCb, this, _1));
	}

	void doneCb(const actionlib::SimpleClientGoalState& state,
				const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		ROS_INFO("[Robot 1 has arrived at its goal]");
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
	bool backup_received;

	geometry_msgs::PoseArray pass_down_frontier_send;
	geometry_msgs::PoseArray pass_down_path_send;
	vector<uint8_t> chosen_queue_send;
	bool backup_send;

	geometry_msgs::PoseStamped chosen_pt;
	frontier_pkg_tb::ChoiceMsg choice_msg;

	MainAction(std::string name) :
		as_(nh_, name, boost::bind(&MainAction::executeCB, this, _1), false),
		action_name_(name)
	{
		as_.start();
		choice_srv = nh_.serviceClient<frontier_pkg_tb::ChoiceMsg>("/choose_frontier_r1");
		choice_pub = nh_.advertise<geometry_msgs::PoseStamped>("/r1_chosen_pt", 1000);
		ROS_INFO("[R1 main action server active]");
	}

	~MainAction(void)
	{
	}

	void executeCB(const r1_main_pkg::PassDownGoalConstPtr &msg){
		ros::Rate(1);
		bool success;
		ROS_INFO("[Robot 1 executing decision process]");

		pass_down_frontier_received = msg->pass_down_frontier_req;
		pass_down_path_received = msg->pass_down_path_req;
		chosen_queue_received = msg->chosen_queue_req;
		robots_remaining_received = msg->robots_remaining;
		backup_received = msg->backup;

		//Setup choice service
		choice_msg.request.pass_down_frontier_req = pass_down_frontier_received;
		choice_msg.request.pass_down_path_req = pass_down_path_received;
		choice_msg.request.chosen_queue_req = chosen_queue_received;
		choice_msg.request.robots_remaining = robots_remaining_received;
		choice_msg.request.backup_req = backup_received;

		if (choice_srv.call(choice_msg)){
			ROS_INFO("[Robot 1 sent request for frontier choice]");

			chosen_pt = choice_msg.response.chosen_pt;
			pass_down_frontier_send = choice_msg.response.pass_down_frontier_res;
			pass_down_path_send = choice_msg.response.pass_down_path_res;
			chosen_queue_send = choice_msg.response.chosen_queue_res;
			backup_send = choice_msg.response.backup_res;

			cout << "size of pass down path: " << pass_down_path_send.poses.size() << endl;
			choice_pub.publish(chosen_pt);

		}

		else{
			ROS_ERROR("[Could not send frontier request for robot 1]");
		}
		PassDown pass_down(pass_down_frontier_send, pass_down_path_send, chosen_queue_send, robots_remaining_received-1, backup_send);
		MoveBase move_base(chosen_pt);

		while (ros::ok()){

			actionlib::SimpleClientGoalState move_base_state = move_base.move_base_r1.getState();
			actionlib::SimpleClientGoalState pass_down_state = pass_down.pass_down_r2.getState();


			if (as_.isPreemptRequested()){
				ROS_INFO("[%s: Preempted]", action_name_.c_str());
				as_.setPreempted();
				success = false;
				break;
			}
			else if (move_base_state.toString() == "SUCCEEDED" && pass_down_state.toString() == "SUCCEEDED"){
				success = true;
				result_.success = success;
				ROS_INFO("[%s: Succeeded]", action_name_.c_str());
				as_.setSucceeded(result_);
				break;

			}
			else{
				continue;
			}

		}

	}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "R1_main_node");

	MainAction main_action("R1_main_action_server");
	ros::spin();

	return 0;
}