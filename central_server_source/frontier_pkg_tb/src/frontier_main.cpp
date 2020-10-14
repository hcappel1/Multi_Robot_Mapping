#include <ros/ros.h>
#include <frontier_pkg_tb/FrontierMsg.h>
#include <frontier_pkg_tb/ChoiceMsgRelay.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "frontier_service_node");
	ros::NodeHandle nh;

	boost::shared_ptr<nav_msgs::OccupancyGrid const> map_msg;
	geometry_msgs::PoseArray frontier_pts;

	//publishers and subscribers
	ros::Publisher frontier_pub = nh.advertise<geometry_msgs::PoseArray>("/optimal_frontier_pts", 1000);

	//service clients
	ros::ServiceClient frontier_client = nh.serviceClient<frontier_pkg_tb::FrontierMsg>("/frontier_pts");
	ros::ServiceClient pass_down_r1 = nh.serviceClient<frontier_pkg_tb::ChoiceMsgRelay>("/R1_main_service");

	//service messages
	frontier_pkg_tb::FrontierMsg frontier_srv;
	frontier_pkg_tb::ChoiceMsgRelay pass_down_msg;

	map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
	frontier_srv.request.map_data = *map_msg;

	if (frontier_client.call(frontier_srv)){
		ROS_INFO("[Sent request for global optimal frontier points]");
		frontier_pts = frontier_srv.response.optimal_frontier_pts;
		frontier_pts.header.stamp = ros::Time::now();
		frontier_pts.header.frame_id = "map";

		ros::Duration(0.5).sleep();
		frontier_pub.publish(frontier_pts);

		//send optimal frontier points to robot 1
		geometry_msgs::PoseArray pass_down_path;
		vector<uint8_t> init_chosen_queue(frontier_pts.poses.size(),'0'); 

		pass_down_msg.request.pass_down_frontier_req = frontier_pts;
		pass_down_msg.request.pass_down_path_req = pass_down_path;
		pass_down_msg.request.chosen_queue_req = init_chosen_queue;
		pass_down_msg.request.robots_remaining = 2;

		if (pass_down_r1.call(pass_down_msg)){
			ROS_INFO("[Passed information to robot 1]");
		}
		else{
			ROS_ERROR("[Could not pass information to robot 1]");
		}



	}

	else {
		ROS_ERROR("Failed to call frontier point service");
		return 1;
	}

	ros::spin();


	return 0;
}