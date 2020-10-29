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


class BackupChoice{
public:
	//ros variables
	ros::NodeHandle nh_;
	ros::ServiceClient get_path_srv;

	//path variables
	geometry_msgs::PoseStamped start_pose;
	geometry_msgs::PoseStamped end_pose;
	nav_msgs::GetPlan plan_msg;
	nav_msgs::Path path;
	double path_distance;

	//tuning variables
	double neighbor_dist_thresh = 5.0;

	//Backup choice function
	shared_ptr<FrontierPt> candidate_pt_backup;
	vector< shared_ptr<FrontierPt> > frontier_queue_;
	int robots_remaining_;
	shared_ptr<FrontierPt> chosen_pt;
	bool backup_send = false;

	//map to odom transform variables
	double x_trans_odom = 9.2;
	double y_trans_odom = 10.0;
	double theta_odom = M_PI/2;

	//odom to map transform variables
	double x_trans_map = 9.7;
	double y_trans_map = 9.0;
	double theta_map = -M_PI/2;

	//message variables
	geometry_msgs::PoseStamped chosen_pt_msg_;
	geometry_msgs::PoseArray pass_down_path_msg_;
	geometry_msgs::PoseArray pass_down_frontier_msg_;
	vector< uint8_t > chosen_queue_msg_;
	bool backup_send_ = true;




	BackupChoice(){
		get_path_srv = nh_.serviceClient<nav_msgs::GetPlan>("tb3_0/move_base/make_plan");
	}

	~BackupChoice(){

	}

	void Initialize(vector< shared_ptr<FrontierPt> > frontier_queue, int robots_remaining, vector<uint8_t> chosen_queue){
		ROS_INFO("[Robot 1 executing backup choice]");
		frontier_queue_ = frontier_queue;
		robots_remaining_ = robots_remaining;
		chosen_queue_msg_ = chosen_queue;
	}

	nav_msgs::Path GetBackupPath(shared_ptr<FrontierPt> candidate_pt_backup)
	{

		start_pose.pose.position.x = candidate_pt_backup->pose.position.x*cos(theta_odom) - candidate_pt_backup->pose.position.y*sin(theta_odom) + x_trans_odom;
		start_pose.pose.position.y = candidate_pt_backup->pose.position.x*sin(theta_odom) + candidate_pt_backup->pose.position.y*cos(theta_odom) - y_trans_odom; 
		start_pose.pose.orientation = candidate_pt_backup->pose.orientation;
		end_pose.pose.position.x = frontier_queue_.back()->pose.position.x*cos(theta_odom) - frontier_queue_.back()->pose.position.y*sin(theta_odom) + x_trans_odom; 
		end_pose.pose.position.y = frontier_queue_.back()->pose.position.x*sin(theta_odom) + frontier_queue_.back()->pose.position.y*cos(theta_odom) - y_trans_odom; 
		end_pose.pose.orientation.w = 1.0;

		start_pose.header.stamp = ros::Time::now();
		start_pose.header.frame_id = "tb3_0/map";
		end_pose.header.stamp = ros::Time::now();
		end_pose.header.frame_id = "tb3_0/map";

		plan_msg.request.start = start_pose;
		plan_msg.request.goal = end_pose;
		plan_msg.request.tolerance = 0.2;

		if (get_path_srv.call(plan_msg)){
			path = plan_msg.response.plan;
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

	void EraseFrontierQueue(shared_ptr<FrontierPt> chosen_pt){

		for (int i = 0; i < frontier_queue_.size(); i++){
			if (chosen_pt->key == frontier_queue_[i]->key){
				frontier_queue_[i]->chosen = true;
			}
		}
	}

	bool BackupFrontierChoice(){

		if (frontier_queue_.size() > 1){
			for (int i = 0; i < frontier_queue_.size(); i++){
				if (frontier_queue_[i]->chosen == false){
					candidate_pt_backup = frontier_queue_[i];
					
					path = GetBackupPath(candidate_pt_backup);
					path_distance = PathDistance(path);

					if (path_distance > robots_remaining_*neighbor_dist_thresh){
						continue;
					}
					else{
						chosen_pt = candidate_pt_backup;

						int robots_use = 1;
						double path_sect = path_distance;
						while (path_sect >= neighbor_dist_thresh){
							robots_use++;
							path_sect = path_sect/robots_use;
						}

						for (int j = 1; j <= robots_use-1; j++){
							
							geometry_msgs::Pose relay_pose = path.poses[int(j*path.poses.size()/robots_use)].pose;
							double x_pos = relay_pose.position.x;
							double y_pos = relay_pose.position.y;
							relay_pose.position.x = x_pos*cos(theta_map) - y_pos*sin(theta_map) + x_trans_map;
							relay_pose.position.y = x_pos*sin(theta_map) + y_pos*cos(theta_map) + y_trans_map;
							pass_down_path_msg_.poses.push_back(relay_pose);

						}

						EraseFrontierQueue(candidate_pt_backup);

						backup_send = true;
						return true;
						break;
					}
				}
				else{
					continue;
				}
			}
		}

		return false;
		
	}

	void ConvertToMessageBackup(){

		path.header.frame_id = "tb3_0/map";

		chosen_pt_msg_.header.frame_id = "map";
		chosen_pt_msg_.pose = chosen_pt->pose;

		pass_down_path_msg_.header.frame_id = "map";
		pass_down_path_msg_.header.stamp = ros::Time::now();

		pass_down_frontier_msg_.header.frame_id = "map";
		for (int i = 0; i < frontier_queue_.size(); i++){
			pass_down_frontier_msg_.poses.push_back(frontier_queue_[i]->pose);
			if (frontier_queue_[i]->chosen == true){
				chosen_queue_msg_[i] = '1';
			}
		}

	}

	void ClearDataBackup(){
		pass_down_path_msg_.poses.clear();
		pass_down_frontier_msg_.poses.clear();
	}

};


class ChooseFrontierMain{

private:
	//ros variables
	ros::NodeHandle nh;
	ros::Publisher frontier_pub;	
	ros::Publisher relay_pose_pub;
	ros::Publisher paths_test_pub;

	BackupChoice backup_choice;

	//service request variables
	geometry_msgs::PoseArray pass_down_frontier;
	geometry_msgs::PoseArray pass_down_path;
	vector< uint8_t > chosen_queue;
	int robots_remaining;
	bool backup_receive;

	//main frontier choice variables
	vector< shared_ptr<FrontierPt> > path_queue;
	vector< shared_ptr<FrontierPt> > frontier_queue;
	vector< shared_ptr<FrontierPt> > valid_frontier_queue;
	vector<vector< shared_ptr<FrontierPt> >> source_paths;
	vector<vector< shared_ptr<FrontierPt> >> filtered_paths;
	vector< shared_ptr<FrontierPt> > optimal_path;
	shared_ptr<FrontierPt> candidate_pt;
	double neighbor_dist_thresh = 5.0;
	bool backup_choice_use = false;

	//service response variables
	shared_ptr<FrontierPt> chosen_pt;
	vector< shared_ptr<FrontierPt> > pass_down_path_res;
	vector< shared_ptr<FrontierPt> > pass_down_frontier_res;

	//message conversion variables
	geometry_msgs::PoseStamped chosen_pt_msg;
	geometry_msgs::PoseArray pass_down_path_msg;
	geometry_msgs::PoseArray pass_down_frontier_msg;
	bool backup_send = false;

	//Initial pose
	shared_ptr<FrontierPt> initial_pose;

	

public:


	ChooseFrontierMain(){

		frontier_pub = nh.advertise<geometry_msgs::PoseStamped>("tb3_0/chosen_frontier_pt", 1000);
		relay_pose_pub = nh.advertise<geometry_msgs::PoseArray>("tb3_0/relay_poses", 1000);
		paths_test_pub = nh.advertise<nav_msgs::Path>("tb3_0/backup_path", 1000);
		
	}

	bool ServiceCallback(frontier_pkg_tb::ChoiceMsg::Request &req,
						frontier_pkg_tb::ChoiceMsg::Response &res)
	{
		pass_down_frontier = req.pass_down_frontier_req;
		pass_down_path = req.pass_down_path_req;
		robots_remaining = req.robots_remaining;
		chosen_queue = req.chosen_queue_req;
		backup_receive = req.backup_req;
		
		InitialPose();
		ChooseFrontierPoint();
		if (backup_choice_use == true){
			backup_choice.ConvertToMessageBackup();
			res.chosen_pt = backup_choice.chosen_pt_msg_;
			res.pass_down_frontier_res = backup_choice.pass_down_frontier_msg_;
			res.pass_down_path_res = backup_choice.pass_down_path_msg_;
			res.chosen_queue_res = backup_choice.chosen_queue_msg_;
			res.backup_res = backup_choice.backup_send_;
			res.success = true;
		}
		else{
			ConvertToMessage();
			res.chosen_pt = chosen_pt_msg;
			res.pass_down_frontier_res = pass_down_frontier_msg;
			res.pass_down_path_res = pass_down_path_msg;
			res.chosen_queue_res = chosen_queue;
			res.backup_res = backup_send;
			res.success = true;
		}


		ClearData();
		backup_choice.ClearDataBackup();
		return true;
	}

	void InitialPose(){
		shared_ptr<FrontierPt> new_frontier_pt = shared_ptr<FrontierPt>( new FrontierPt );
		new_frontier_pt->pose.position.x = 9.0;
		new_frontier_pt->pose.position.y = 13.17;
		new_frontier_pt->pose.orientation.w = 1.0;
		new_frontier_pt->key = (to_string(new_frontier_pt->pose.position.x) + "-" + to_string(new_frontier_pt->pose.position.y));
		initial_pose = new_frontier_pt;
	}

	void CreateFrontierQueue(){

		for (int i = 0; i < pass_down_frontier.poses.size(); i++){

			shared_ptr<FrontierPt> new_frontier_pt = shared_ptr<FrontierPt>( new FrontierPt );
			new_frontier_pt->pose = pass_down_frontier.poses[i];
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
			shared_ptr<FrontierPt> new_frontier_pt = shared_ptr<FrontierPt>( new FrontierPt );
			new_frontier_pt->pose = pass_down_path.poses[i];
			new_frontier_pt->key = (to_string(pass_down_path.poses[i].position.x) + "-" + to_string(pass_down_path.poses[i].position.y));
			new_frontier_pt->chosen = false;
			path_queue.push_back(new_frontier_pt);
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


	void ChooseFrontierPoint()
	{
		CreateFrontierQueue();
		GetNeighbors();
		CreatePathQueue();


		if (pass_down_path.poses.size() > 0){

			chosen_pt = path_queue[0];
			PopFront(path_queue);

			EraseFrontierQueue(chosen_pt);
			
			pass_down_path_res = path_queue;
			pass_down_frontier_res = frontier_queue;
		}
		else{
			bool ret = false;

			for (int i = 0; i < frontier_queue.size()-1; i++){
				if (FrontierValid(frontier_queue[i]) == true){
					valid_frontier_queue.push_back(frontier_queue[i]);
				}
			}

			cout << "Size of frontier queue: " << frontier_queue.size() << endl;
			cout << "Size of valid frontier_queue: " << valid_frontier_queue.size() << endl;

			while (valid_frontier_queue.size() > 0){
				if (valid_frontier_queue[0]->chosen == false){
					candidate_pt = valid_frontier_queue[0];
					ComputePaths(candidate_pt);
					FilterPaths();

					if (filtered_paths.size() > 0){
						ROS_INFO("[R1 found a valid point]");
						OptimalPath();
						PopFront(optimal_path);
						optimal_path.pop_back();
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

				backup_choice.Initialize(frontier_queue, robots_remaining, chosen_queue);
				if (backup_choice.BackupFrontierChoice()){
					backup_choice_use = true;
					backup_choice.ConvertToMessageBackup();
					paths_test_pub.publish(backup_choice.path);
					relay_pose_pub.publish(backup_choice.pass_down_path_msg_);
				}
				else{

					ROS_INFO("[Robot 1 failed to find a valid point. Returning to initial pose.]");
					chosen_pt = initial_pose;
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
		for (int i = 0; i < pass_down_path_res.size(); i++){
			pass_down_path_msg.poses.push_back(pass_down_path_res[i]->pose);
		}
		
		pass_down_frontier_msg.header.stamp = ros::Time::now();
		pass_down_frontier_msg.header.frame_id = "map";
		for (int j = 0; j < pass_down_frontier_res.size()-1; j++){
			pass_down_frontier_msg.poses.push_back(pass_down_frontier_res[j]->pose);

			if (pass_down_frontier_res[j]->chosen == true){
				chosen_queue[j] = '1';
			}
		}

	}

	void ClearData(){
		path_queue.clear();
		frontier_queue.clear();
		valid_frontier_queue.clear();
		source_paths.clear();
		filtered_paths.clear();
		optimal_path.clear();
		pass_down_path_res.clear();
		pass_down_frontier_res.clear();
		pass_down_path_msg.poses.clear();
		pass_down_frontier_msg.poses.clear();
		backup_choice_use = false;
	}



};


int main(int argc, char **argv){

	ros::init(argc, argv, "frontier_choice_node_r1");
	ros::NodeHandle nh_;

	ChooseFrontierMain choose_frontier_main;
	ros::ServiceServer choice_service = nh_.advertiseService("/choose_frontier_r1", &ChooseFrontierMain::ServiceCallback, &choose_frontier_main);

	ros::spin();
	return 0;
}