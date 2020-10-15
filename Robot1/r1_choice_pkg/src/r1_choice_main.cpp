#include <ros/ros.h>
#include <frontier_pkg_tb/ChoiceMsgRelay.h>
#include <frontier_pkg_tb/ChoiceMsg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

class Main{
public:
	ros::NodeHandle nh_;
	ros::ServiceClient choice_srv;
	ros::Publisher chosen_pt_pub;
	frontier_pkg_tb::ChoiceMsg choice_msg;
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

	//choice decision variables
	geometry_msgs::PoseStamped chosen_pt;
	geometry_msgs::PoseArray pass_down_frontier_received;
	geometry_msgs::PoseArray pass_down_path_received;
	vector<uint8_t> chosen_queue_received;

	Main(){
		ROS_INFO("[R1 main class constructed]");
		choice_srv = nh_.serviceClient<frontier_pkg_tb::ChoiceMsg>("/choose_frontier_r1");
		chosen_pt_pub = nh_.advertise<geometry_msgs::PoseStamped>("R1_chosen_frontier_pt", 1000);
	}


	bool ServiceCallback(frontier_pkg_tb::ChoiceMsgRelay::Request &req,
						 frontier_pkg_tb::ChoiceMsgRelay::Response &res){
		choice_msg.request.pass_down_frontier_req = req.pass_down_frontier_req;
		choice_msg.request.pass_down_path_req = req.pass_down_path_req;
		choice_msg.request.chosen_queue_req = req.chosen_queue_req;
		choice_msg.request.robots_remaining = req.robots_remaining;

		if (choice_srv.call(choice_msg)){
			ROS_INFO("[Robot 1 sent request for frontier choice]");

			chosen_pt = choice_msg.response.chosen_pt;
			pass_down_frontier_received = choice_msg.response.pass_down_frontier_res;
			pass_down_path_received = choice_msg.response.pass_down_path_res;
			chosen_queue_received = choice_msg.response.chosen_queue_res;

			cout << "size of pass down path: " << pass_down_path_received.poses.size() << endl;
			chosen_pt_pub.publish(chosen_pt);

		}

		else{
			ROS_ERROR("[Could not send frontier request for robot 1]");
		}
		MoveBaseClient move_base_r1("tb3_0/move_base", true);
		move_base_r1.waitForServer();

		move_base_msgs::MoveBaseGoal nav_goal;
		nav_goal.target_pose = chosen_pt;
		nav_goal.target_pose.pose.orientation.w = 1.0;

		move_base_r1.sendGoal(nav_goal);

		move_base_r1.waitForResult();


		res.success = true;


	}

	void doneCb(const actionlib::SimpleClientGoalState& state,
				const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		ROS_INFO("[Robot 1 has arrived at its goal]");
	}

	void activeCb()
	{
		ROS_INFO("[Goal is active]");
	}

	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){

	}
};



int main(int argc, char **argv){

	ros::init(argc, argv, "R1_main_node");
	ros::NodeHandle nh;

	Main main;

	ros::ServiceServer r1_main_service = nh.advertiseService("/R1_main_service", &Main::ServiceCallback, &main);

	ros::spin();

	return 0;
}