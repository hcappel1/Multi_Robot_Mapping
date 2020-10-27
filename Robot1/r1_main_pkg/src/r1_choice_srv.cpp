#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <frontier_pkg_tb/ChoiceMsg.h>
#include <vector>
#include <iostream>
#include <memory>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <list>
#include <algorithm>
#include <nav_msgs/GetPlan.h>

using namespace std;

class FrontierPt{

public:

	FrontierPt(){
		SourceInitialization();
	}

	geometry_msgs::Pose pose;
	vector< shared_ptr<FrontierPt> > neighbors;
	string key; 
	bool chosen;
	int rank_val;

	void SourceInitialization(){
		pose.position.x = 11.129;
		pose.position.y = 14.2085;
		chosen = false;
		key = "source";
	}
};

class ChooseFrontier{

private:
	ros::NodeHandle nh;
	ros::Publisher frontier_pub;
	ros::Publisher frontier_pts_pub;
	ros::Publisher valid_frontier_pts_pub;
	ros::Publisher paths_test_pub;
	ros::ServiceClient get_path_srv;

	geometry_msgs::PoseArray pass_down_frontier;
	geometry_msgs::PoseArray pass_down_path;
	vector< uint8_t > chosen_queue;
	vector< shared_ptr<FrontierPt> > pass_down_path_vec;
	vector< shared_ptr<FrontierPt> > frontier_queue;
	vector< shared_ptr<FrontierPt> > valid_frontier_queue;
	vector<vector< shared_ptr<FrontierPt> >> source_paths;
	vector<vector< shared_ptr<FrontierPt> >> filtered_paths;
	vector< shared_ptr<FrontierPt> > optimal_path;
	geometry_msgs::PoseArray optimal_path_res;
	int robots_remaining;

	shared_ptr<FrontierPt> candidate_pt;

	//service response variables
	shared_ptr<FrontierPt> chosen_pt;
	vector< shared_ptr<FrontierPt> > pass_down_path_res;
	vector< shared_ptr<FrontierPt> > pass_down_frontier_res;

	//message conversion variables
	geometry_msgs::PoseStamped chosen_pt_msg;
	geometry_msgs::PoseArray pass_down_path_msg;
	geometry_msgs::PoseArray pass_down_frontier_msg;

	//tuning variables
	double neighbor_dist_thresh = 5.0;

	//Backup choice function
	shared_ptr<FrontierPt> candidate_pt_backup;
	geometry_msgs::PoseStamped start_pose;
	geometry_msgs::PoseStamped end_pose;
	nav_msgs::GetPlan plan_msg;
	ros::Publisher start_pose_pub;
	ros::Publisher end_pose_pub;
	ros::Publisher relay_pose_pub;
	bool backup_receive;
	bool backup_send = true;
	


public:


	ChooseFrontier(){

		frontier_pub = nh.advertise<geometry_msgs::PoseStamped>("chosen_frontier_pt", 1000);
		frontier_pts_pub = nh.advertise<geometry_msgs::PoseArray>("frontier_queue", 1000);
		valid_frontier_pts_pub = nh.advertise<geometry_msgs::PoseArray>("valid_frontier_queue", 1000);
		paths_test_pub = nh.advertise<nav_msgs::Path>("paths_test", 1000);
		get_path_srv = nh.serviceClient<nav_msgs::GetPlan>("tb3_0/move_base/make_plan");

		start_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("start_pose", 1000);
		end_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("end_pose", 1000);
		relay_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("relay_pose", 1000);
		


	}

	bool ServiceCallback(frontier_pkg_tb::ChoiceMsg::Request &req,
						frontier_pkg_tb::ChoiceMsg::Response &res)
	{
		pass_down_frontier = req.pass_down_frontier_req;
		pass_down_path = req.pass_down_path_req;
		robots_remaining = req.robots_remaining;
		chosen_queue = req.chosen_queue_req;
		backup_receive = req.backup_req;
		
		//ChooseFrontierPoint();
		BackupFrontierChoice();
		ConvertToMessage();

		res.chosen_pt = chosen_pt_msg;
		res.pass_down_frontier_res = pass_down_frontier_msg;
		res.pass_down_path_res = pass_down_path_msg;
		res.chosen_queue_res = chosen_queue;
		res.backup_res = backup_send;
		res.success = true;

		ClearData();
		return true;
	}

	void CreateFrontierQueue(){

		for (int i = 0; i < pass_down_frontier.poses.size(); i++){

			shared_ptr<FrontierPt> new_frontier_pt = shared_ptr<FrontierPt>( new FrontierPt );
			new_frontier_pt->pose = pass_down_frontier.poses[i];
			//new_frontier_pt->pose.orientation.w = 1.0;
			new_frontier_pt->key = (to_string(pass_down_frontier.poses[i].position.x) + "-" + to_string(pass_down_frontier.poses[i].position.y));
			new_frontier_pt->rank_val = i + 1; 
			if (chosen_queue[i] == '1'){
				new_frontier_pt->chosen = true;
			}
			frontier_queue.push_back(new_frontier_pt);
		}
		shared_ptr<FrontierPt> source = shared_ptr<FrontierPt>( new FrontierPt );
		source->rank_val = pass_down_frontier.poses.size() + 2;
		frontier_queue.push_back(source);
	}

	void CreatePathQueue(){

		for (int i = 0; i < pass_down_path.poses.size(); i++){
			for (int j = 0; j < frontier_queue.size(); j++){
				if (abs(pass_down_path.poses[i].position.x - frontier_queue[j]->pose.position.x) < 0.0001 && abs(pass_down_path.poses[i].position.y - frontier_queue[j]->pose.position.y) < 0.0001){
					pass_down_path_vec.push_back(frontier_queue[j]);
				}
			}
		}
	}

	void GetNeighbors(){

		for (int i = 0; i < frontier_queue.size(); i++){
			for (int j = 0; j < frontier_queue.size(); j++){

				if (frontier_queue[i]->key != frontier_queue[j]->key){

					double distance = sqrt(pow(frontier_queue[i]->pose.position.x - frontier_queue[j]->pose.position.x,2) + pow(frontier_queue[i]->pose.position.y - frontier_queue[j]->pose.position.y,2));
					if (distance < neighbor_dist_thresh){
						frontier_queue[i]->neighbors.push_back(frontier_queue[j]);
					}
				}
			}
		}
	}

	bool FrontierValid( shared_ptr<FrontierPt> frontier_pt){

		for (int i = 0; i < frontier_pt->neighbors.size(); i++){

			if (frontier_pt->neighbors[i]->key == "source"){
				return true;
			}
			else{
				for (int j = 0; j < frontier_pt->neighbors[i]->neighbors.size(); j++){

					if (frontier_pt->neighbors[i]->neighbors[j]->key == "source"){
						return true;
					}
					else{
						for (int k = 0; k < frontier_pt->neighbors[i]->neighbors[j]->neighbors.size(); k++){
							if (frontier_pt->neighbors[i]->neighbors[j]->neighbors[k]->key == "source"){
								return true;
							}
							else{
								continue;
							}
						}
					}
				}
			}
			
		}
		return false;
	}

	void ComputePaths( shared_ptr<FrontierPt> candidate_pt){
		vector< shared_ptr<FrontierPt> > source_path;
		source_path.push_back(candidate_pt);

		for (int i = 0; i < candidate_pt->neighbors.size(); i++){

			vector< shared_ptr<FrontierPt> > source_path_one = source_path;
			source_path_one.push_back(candidate_pt->neighbors[i]);

			if (candidate_pt->neighbors[i]->key == "source"){
				source_paths.push_back(source_path_one);
			}
			
			for (int j = 0; j < candidate_pt->neighbors[i]->neighbors.size(); j++){

				vector< shared_ptr<FrontierPt> > source_path_two = source_path_one;
				source_path_two.push_back(candidate_pt->neighbors[i]->neighbors[j]);

				if (candidate_pt->neighbors[i]->neighbors[j]->key == "source"){
					source_paths.push_back(source_path_two);
				}

				for (int k = 0; k < candidate_pt->neighbors[i]->neighbors[j]->neighbors.size(); k++){

					vector< shared_ptr<FrontierPt> > source_path_three = source_path_two;
					source_path_three.push_back(candidate_pt->neighbors[i]->neighbors[j]->neighbors[k]);
					
					if (candidate_pt->neighbors[i]->neighbors[j]->neighbors[k]->key == "source"){
						source_paths.push_back(source_path_three);
				
					}
				}
			}
		}
	}

	void FilterPaths(){

		for (int i = 0; i < source_paths.size(); i++){
			if (source_paths[i].size() <= robots_remaining + 1){
				filtered_paths.push_back(source_paths[i]);
			}
		}
	}

	void OptimalPath(){
		double current_score = 100;
		double node_val = 1.0;

		for (int i = 0; i < filtered_paths.size(); i++){
			double score = 0.0;
			for (int j = 0; j < filtered_paths[i].size(); j++){
				score += (node_val + filtered_paths[i][j]->rank_val);
			}
			if (score < current_score){
				optimal_path = filtered_paths[i];
				current_score = score;
			}
			else{
				continue;
			}
		}
	}

	void EraseFrontierQueue(shared_ptr<FrontierPt> chosen_pt){

		for (int i = 0; i < frontier_queue.size(); i++){
			if (chosen_pt->key == frontier_queue[i]->key){
				frontier_queue[i]->chosen = true;
			}
		}
	}

	void PopFront(vector< shared_ptr<FrontierPt> > &input_vec){
		
		if (input_vec.size() > 0){
			input_vec.erase(input_vec.begin());
		}
	}

	nav_msgs::Path GetBackupPath(shared_ptr<FrontierPt> candidate_pt_backup)
	{
		nav_msgs::Path path;

		//map transform params from odom
		double x_trans = 9.2;
		double y_trans = 10.0;
		double theta = M_PI/2;

		start_pose.pose.position.x = candidate_pt_backup->pose.position.x*cos(theta) - candidate_pt_backup->pose.position.y*sin(theta) + x_trans;
		start_pose.pose.position.y = candidate_pt_backup->pose.position.x*sin(theta) + candidate_pt_backup->pose.position.y*cos(theta) - y_trans; 
		start_pose.pose.orientation = candidate_pt_backup->pose.orientation;
		end_pose.pose.position.x = frontier_queue.back()->pose.position.x*cos(theta) - frontier_queue.back()->pose.position.y*sin(theta) + x_trans; 
		end_pose.pose.position.y = frontier_queue.back()->pose.position.x*sin(theta) + frontier_queue.back()->pose.position.y*cos(theta) - y_trans; 
		end_pose.pose.orientation.w = 1.0;

		start_pose.header.stamp = ros::Time::now();
		start_pose.header.frame_id = "tb3_0/map";
		end_pose.header.stamp = ros::Time::now();
		end_pose.header.frame_id = "tb3_0/map";

		plan_msg.request.start = start_pose;
		plan_msg.request.goal = end_pose;
		plan_msg.request.tolerance = 0.2;

		if (get_path_srv.call(plan_msg)){
			ROS_INFO("[Sent for backup choice path]");
			path = plan_msg.response.plan;

			cout << "Size of path: " << path.poses.size() << endl;
			return path;
		}
		else{
			ROS_ERROR("[Could not get backup choice path]");
		}
	}

	double PathDistance(nav_msgs::Path path){

		double path_distance = 0.0;
		double x_pos = path.poses[0].pose.position.x;
		double y_pos = path.poses[0].pose.position.y;

		for (int i = 1; i < path.poses.size(); i++){
			path_distance += sqrt(pow(path.poses[i].pose.position.x - x_pos,2) + pow(path.poses[i].pose.position.y - y_pos,2));
			x_pos = path.poses[i].pose.position.x;
			y_pos = path.poses[i].pose.position.y;
		}

		return path_distance;	
	}

	bool BackupFrontierChoice(){

		nav_msgs::Path path;
		double path_distance;
		double path_sect;
		double x_trans = 9.7;
		double y_trans = 9.0;
		double theta = -M_PI/2;

		CreateFrontierQueue();
		GetNeighbors();
		CreatePathQueue();

		ROS_INFO("Robot 1 executing backup function");

		for (int i = 0; i < frontier_queue.size(); i++){
			if (frontier_queue[i]->chosen == false){
				candidate_pt_backup = frontier_queue[i];
				
				path = GetBackupPath(candidate_pt_backup);
				path_distance = PathDistance(path);

				if (path_distance > robots_remaining*neighbor_dist_thresh){
					continue;
				}
				else{
					chosen_pt = candidate_pt_backup;

					for (int j = 1; j <= robots_remaining-1; j++){
						
						geometry_msgs::Pose relay_pose = path.poses[int(path.poses.size()/2)].pose;
						double x_pos = relay_pose.position.x;
						double y_pos = relay_pose.position.y;
						relay_pose.position.x = x_pos*cos(theta) - y_pos*sin(theta) + x_trans;
						relay_pose.position.y = x_pos*sin(theta) + y_pos*cos(theta) + y_trans;
						pass_down_path_msg.poses.push_back(relay_pose);
						// geometry_msgs::PoseStamped relay_pose_publish;
						// relay_pose_publish.header.frame_id = "map";
						// relay_pose_publish.pose = relay_pose;
						// relay_pose_publish.pose.orientation.w = 1.0;
						// relay_pose_pub.publish(relay_pose_publish);
					}

					path.header.frame_id = "tb3_0/map";
					paths_test_pub.publish(path);

					start_pose_pub.publish(start_pose);
					end_pose_pub.publish(end_pose);

					EraseFrontierQueue(candidate_pt_backup);
					pass_down_frontier_res = frontier_queue;

					backup_send = true;
					return true;
					break;
				}
			}
			else{
				continue;
			}
		}

		return false;
		
	}


	void ChooseFrontierPoint()
	{
		CreateFrontierQueue();
		GetNeighbors();
		CreatePathQueue();


		if (pass_down_path.poses.size() > 1){
			chosen_pt = pass_down_path_vec[0];
			PopFront(pass_down_path_vec);
			EraseFrontierQueue(chosen_pt);

			pass_down_path_res = pass_down_path_vec;
			pass_down_frontier_res = frontier_queue;
		}
		else{
			bool ret = false;

			for (int i = 0; i < frontier_queue.size(); i++){
				if (FrontierValid(frontier_queue[i]) == true){
					valid_frontier_queue.push_back(frontier_queue[i]);
				}
			}
			cout << "Size of frontier queue: " << frontier_queue.size() << endl;
			cout << "Size of valid frontier queue: " << valid_frontier_queue.size() << endl;

			while (valid_frontier_queue.size() > 0){
				if (valid_frontier_queue[0]->chosen == false){
					candidate_pt = valid_frontier_queue[0];
					ComputePaths(candidate_pt);
					FilterPaths();

					if (filtered_paths.size() > 0){
						cout << "found a valid chosen point" << endl;
						OptimalPath();
						PopFront(optimal_path);
						EraseFrontierQueue(candidate_pt);
						chosen_pt = candidate_pt;
						pass_down_path_res = optimal_path;
						pass_down_frontier_res = frontier_queue;
						ret = true;
						break;
					}
					else{

						PopFront(valid_frontier_queue);
						source_paths.clear();
						filtered_paths.clear();

					}
				}
				else{
					PopFront(valid_frontier_queue);
				}
			}
			if (ret == false){

				if (BackupFrontierChoice()){
					ROS_INFO("[Robot 1 found a backup plan]");
				}
				else{
					ROS_INFO("[Robot 1 failed to find a valid point]");
					chosen_pt = frontier_queue.back();
					pass_down_path_res = optimal_path;
					pass_down_frontier_res = frontier_queue;
				}
			}
		}
	}

	void ConvertToMessage(){

		chosen_pt_msg.header.stamp = ros::Time::now();
		chosen_pt_msg.header.frame_id = "map";
		chosen_pt_msg.pose = chosen_pt->pose;

		pass_down_path_msg.header.stamp = ros::Time::now();
		pass_down_path_msg.header.frame_id = "map";
		if (backup_receive == false){
			for (int i = 0; i < pass_down_path_res.size(); i++){
				pass_down_path_msg.poses.push_back(pass_down_path_res[i]->pose);
			}
		}
		else{
			pass_down_path_msg.poses.push_back(frontier_queue.back()->pose);
		}

		pass_down_frontier_msg.header.stamp = ros::Time::now();
		pass_down_frontier_msg.header.frame_id = "map";
		for (int j = 0; j < pass_down_frontier_res.size(); j++){
			pass_down_frontier_msg.poses.push_back(pass_down_frontier_res[j]->pose);

			if (pass_down_frontier_res[j]->chosen == true){
				chosen_queue[j] = '1';
			}
		}

	}

	void ClearData(){
		pass_down_path_vec.clear();
		frontier_queue.clear();
		valid_frontier_queue.clear();
		source_paths.clear();
		filtered_paths.clear();
		optimal_path.clear();
		optimal_path_res.poses.clear();
		pass_down_path_res.clear();
		pass_down_frontier_res.clear();
		pass_down_path_msg.poses.clear();
		pass_down_frontier_msg.poses.clear();
	}

	void NeighborsTest(){

		geometry_msgs::PoseStamped chosen_frontier_pt;
		geometry_msgs::PoseArray neighbor_frontier_pts;

		chosen_frontier_pt.pose = frontier_queue[0]->pose;
		chosen_frontier_pt.header.stamp = ros::Time::now();
		chosen_frontier_pt.header.frame_id = "map";
		neighbor_frontier_pts.header.stamp = ros::Time::now();
		neighbor_frontier_pts.header.frame_id = "map";

		for (int i = 0; i < frontier_queue[0]->neighbors.size(); i++){
			neighbor_frontier_pts.poses.push_back(frontier_queue[0]->neighbors[i]->pose);
		}

		while (ros::ok()){
			frontier_pub.publish(chosen_frontier_pt);
			//neighbors_pub.publish(neighbor_frontier_pts);
		}
	}

	void FrontierValidTest(){

		geometry_msgs::PoseArray valid_frontier_points;
		valid_frontier_points.header.stamp = ros::Time::now();
		valid_frontier_points.header.frame_id = "map";
		pass_down_frontier.header.stamp = ros::Time::now();
		pass_down_frontier.header.frame_id = "map";



		for (int i = 0; i < frontier_queue.size(); i++){
			bool valid = FrontierValid(frontier_queue[i]);
			if (valid == true){
				cout << "frontier point valid" << endl;
				valid_frontier_points.poses.push_back(frontier_queue[i]->pose);
			}
		}

		while (ros::ok()){
			frontier_pts_pub.publish(pass_down_frontier);
			valid_frontier_pts_pub.publish(valid_frontier_points);
		}


	}

	void ComputePathTest( vector< shared_ptr<FrontierPt> > source_path){
		nav_msgs::Path frontier_path;
		frontier_path.header.stamp = ros::Time::now();
		frontier_path.header.frame_id = "map";


		for (int i = 0; i < source_path.size(); i++){
			geometry_msgs::PoseStamped path_pose;
			path_pose.header.stamp = ros::Time::now();
			path_pose.header.frame_id = "map";
			cout << "source path pose: " << source_path[i]->pose.position.x << "," << source_path[i]->pose.position.y << endl;
			path_pose.pose = source_path[i]->pose;
			frontier_path.poses.push_back(path_pose);
		}

		while (ros::ok()){
			paths_test_pub.publish(frontier_path);
		}

	}




};


int main(int argc, char **argv){

	ros::init(argc, argv, "frontier_choice_node_r1");
	ros::NodeHandle nh_;

	ChooseFrontier choose_frontier;
	ros::ServiceServer choice_service = nh_.advertiseService("/choose_frontier_r1", &ChooseFrontier::ServiceCallback, &choose_frontier);

	ros::spin();
	return 0;
}