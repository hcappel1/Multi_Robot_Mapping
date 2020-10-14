#include <ros/ros.h>
#include <frontier_pkg_tb/ChoiceMsgRelay.h>
#include <frontier_pkg_tb/ChoiceMsg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

class Main{
public:
	ros::NodeHandle nh_;
	ros::ServiceClient choice_srv;
	frontier_pkg_tb::ChoiceMsg choice_msg;

	//choice decision variables
	geometry_msgs::PoseStamped chosen_pt;
	geometry_msgs::PoseArray pass_down_frontier_received;
	geometry_msgs::PoseArray pass_down_path_received;
	vector<uint8_t> chosen_queue_received;

	Main(){
		ROS_INFO("[R1 main class constructed]");
		choice_srv = nh_.serviceClient<frontier_pkg_tb::ChoiceMsg>("/choose_frontier_r1");
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

		}

		else{
			ROS_ERROR("[Could not send frontier request for robot 1]");
		}

		res.success = true;


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