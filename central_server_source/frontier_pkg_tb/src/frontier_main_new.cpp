#include <ros/ros.h>
#include <frontier_pkg_tb/FrontierMsg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <r1_main_pkg/PassDownAction.h>

using namespace std;

class FrontierPts{
public:

	ros::NodeHandle nh_;
	boost::shared_ptr<nav_msgs::OccupancyGrid const> map_msg;
	geometry_msgs::PoseArray frontier_pts_;

	ros::Publisher frontier_pub;
	ros::ServiceClient frontier_client;

	frontier_pkg_tb::FrontierMsg frontier_srv;

	FrontierPts(){
		frontier_pub = nh_.advertise<geometry_msgs::PoseArray>("/global_frontier_pts", 1000);
		frontier_client = nh_.serviceClient<frontier_pkg_tb::FrontierMsg>("/frontier_pts");
		GetFrontierPts();
	}

	void GetFrontierPts(){
		map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
		frontier_srv.request.map_data = *map_msg;

		if (frontier_client.call(frontier_srv)){
			ROS_INFO("[Sent request for global optimal frontier points]");
			frontier_pts_ = frontier_srv.response.optimal_frontier_pts;
			frontier_pts_.header.stamp = ros::Time::now();
			frontier_pts_.header.frame_id = "map";

			ros::Duration(0.5);
			frontier_pub.publish(frontier_pts_);
		}
		else{
			ROS_ERROR("[Failed to call global frontier point service]");
		}
	}
};

class PassDown{
public:
	typedef actionlib::SimpleActionClient<r1_main_pkg::PassDownAction> PassDownClient;
	PassDownClient pass_down_r1;
	geometry_msgs::PoseArray pass_down_frontier_send;
	geometry_msgs::PoseArray pass_down_path_send;

	PassDown(geometry_msgs::PoseArray frontier_pts_) : pass_down_r1("R1_main_action_server")
	{
		pass_down_frontier_send = frontier_pts_;
		SendActionRequest();
	}

	void SendActionRequest(){
		pass_down_r1.waitForServer();

		r1_main_pkg::PassDownGoal msg;
		vector<uint8_t> init_chosen_queue(pass_down_frontier_send.poses.size(),'0');
		msg.pass_down_frontier_req = pass_down_frontier_send;
		msg.pass_down_path_req = pass_down_path_send;
		msg.chosen_queue_req = init_chosen_queue;
		msg.robots_remaining = 2;

		pass_down_r1.sendGoal(msg, boost::bind(&PassDown::doneCb, this, _1, _2), boost::bind(&PassDown::activeCb, this));
	}

	void doneCb(const actionlib::SimpleClientGoalState& state,
				const r1_main_pkg::PassDownResultConstPtr& result)
	{
		ROS_INFO("[Robot 1 has completed]");
	}

	void activeCb()
	{
		ROS_INFO("[Information has been passed to robot 1]");
	}

};



int main(int argc, char **argv){
	ros::init(argc, argv, "CC_main_node");

	FrontierPts frontier_pts;
	PassDown pass_down(frontier_pts.frontier_pts_);

	while (ros::ok()){
		actionlib::SimpleClientGoalState pass_down_state = pass_down.pass_down_r1.getState();

		if (pass_down_state.toString() == "SUCCEEDED"){
			ROS_INFO("[Robot 1 has full completion]");
			break;
		}
	}

	ros::spin();

	return 0;
}

