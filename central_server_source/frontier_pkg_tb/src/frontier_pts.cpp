#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <iostream>
#include <memory>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <frontier_pkg_tb/FrontierMsg.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <list>
#include <algorithm>
#include </home/hcappel1/eigen/Eigen/Dense>


using namespace std;

class Node{

    public:

        Node(){
        	CommonInitialization();
        } 
        
        geometry_msgs::Pose pose;
        string key;
        bool map_open;
        bool map_closed;
        bool frontier_open;
        bool frontier_closed;
        int map_val;
        vector< shared_ptr<Node>> neighbors;
        int row;
        int col;
        double optimal_heuristic;

        void CommonInitialization(){
        	pose.position.x = 0.0;
        	pose.position.y = 0.0;
        	key = (to_string(pose.position.x) + "-" + to_string(pose.position.y));
        	map_open = false;
        	map_closed = false;
        	frontier_open = false;
        	frontier_closed = false;
        }

};

class Frontier{

private:

	int width = 383;
	int height = 387;
	double map_width = 19.20;
	double map_height = 19.43;

	//map transform params from odom
	double x_trans = map_width/2;
	double y_trans = map_height/2;
	double theta = -1.57;

	//frontier filtering information
	int frontier_thresh = 15;
	// Eigen::MatrixXf map_odom_transform(3, 3);
	// map_odom_transform << cos(theta), -sin(theta), x_trans, -sin(theta), cos(theta), y_trans, 0.0, 0.0, 1.0;
	// map_odom_transform(0,0) = cos(theta);
	// map_odom_transform(0,1) = -sin(theta);
	// map_odom_transform(0,2) = x_trans;
	// map_odom_transform(1,0) = -sin(theta);
	// map_odom_transform(1,1) = cos(theta);
	// map_odom_transform(1,2) = y_trans;
	// map_odom_transform(2,0) = 0.0;
	// map_odom_transform(2,1) = 0.0;
	// map_odom_transform(2,2) = 1.0;


	ros::NodeHandle nh;
	ros::Publisher frontier_pub;
	ros::Publisher optimal_pub;
	ros::Subscriber	odom_sub;

	vector< shared_ptr<Node> > map_node;

    list< shared_ptr<Node> > map_queue;
    list< shared_ptr<Node> > frontier_queue; 
    vector< shared_ptr<Node> > new_frontier;

    shared_ptr<Node> current_node_map;
    shared_ptr<Node> current_node_frontier;
    shared_ptr<Node> neighbor_node_map;
    shared_ptr<Node> neighbor_node_frontier;

    vector<vector< shared_ptr<Node> > > frontier_list;
    boost::shared_ptr<nav_msgs::Odometry const> init_pose;
    nav_msgs::Odometry init_pose_obj;



public:

	vector<signed char> map_raw;
	geometry_msgs::PoseArray pose_array;
	geometry_msgs::PoseArray frontier_array;
	geometry_msgs::PoseArray optimal_frontier_pts;
	geometry_msgs::Pose node_pose;
	geometry_msgs::PoseStamped optimal_frontier_pt;
	
	Frontier(){
		ROS_INFO("created frontier object");
		frontier_pub = nh.advertise<geometry_msgs::PoseArray>("frontier_pts", 1000);
		optimal_pub = nh.advertise<geometry_msgs::PoseStamped>("optimal_frontier_pt", 1000);
	}



	bool MapCallback(frontier_pkg_tb::FrontierMsg::Request &req,
					 frontier_pkg_tb::FrontierMsg::Response &res)
	{
		map_raw = req.map_data.data;

		DoFrontier();
		SortFrontierArray();
		OptimalFrontierPts();
		//FrontierArrayTest(frontier_list);

		res.optimal_frontier_pts = optimal_frontier_pts;
		res.success = true;

		ClearData();

		return true;
	}

	void SetInitPose(){
		init_pose = ros::topic::waitForMessage<nav_msgs::Odometry>("/tb3_0/odom");
		init_pose_obj = *init_pose;

		init_pose_obj.pose.pose.position.x = 11.129;
		init_pose_obj.pose.pose.position.y = 13.2085;

	}

	void MapConvert()
	{
		int row_iter = 0;
		int col_iter = 0;
		double x_disc = map_width/width;
		double y_disc = map_height/height;
		int iter = 0;
		
	    for (int i = 0; i < map_raw.size(); i++){

		    if (i == 0){
		   
		    }
		    else if (i % width == 0){
		    	row_iter++;
		    	col_iter = 0;
		    }
		    else{
		    	col_iter++;
		    }


		    shared_ptr<Node> new_node = shared_ptr<Node>( new Node );
		    new_node->pose.position.x = col_iter*x_disc;
		    new_node->pose.position.y = row_iter*y_disc;
		    new_node->pose.position.z = 0.0;
		    new_node->pose.orientation.x = 0.0;
		    new_node->pose.orientation.y = 0.0;
		    new_node->pose.orientation.z = 0.0;
		    new_node->pose.orientation.w = 1.0;
		    new_node->key = (to_string(new_node->pose.position.x) + "-" + to_string(new_node->pose.position.y));
		    new_node->map_val = int(map_raw[i]);
		    new_node->row = row_iter;
		    new_node->col = col_iter;
		    map_node.push_back(new_node);
		    node_pose = new_node->pose;

		    pose_array.poses.push_back(new_node->pose);
	    }


	}

	void PoseSnapInit(){


		for (vector< shared_ptr<Node> >::iterator it = map_node.begin(); it!=map_node.end(); ++it){
			shared_ptr<Node> node_iter = *it;

			if (sqrt(pow(node_iter->pose.position.x - init_pose_obj.pose.pose.position.x,2) + pow(node_iter->pose.position.y - init_pose_obj.pose.pose.position.y,2)) < 0.1){
				current_node_map = node_iter;
				break;
			}
			else{
				continue;
			}
		}
	}

	void GetNeighbors()
	{
		for (int i = 0; i < map_node.size(); i++){
			//check the rows
			if (map_node[i]->row == 0){
				map_node[i]->neighbors.push_back(map_node[i+width]);
			}
			else if (map_node[i]->row == height){
				map_node[i]->neighbors.push_back(map_node[i-width]);
			}
			else{
				map_node[i]->neighbors.push_back(map_node[i+width]);
				map_node[i]->neighbors.push_back(map_node[i-width]);
			}
			//check the columns
			if (map_node[i]->col == 0){
				map_node[i]->neighbors.push_back(map_node[i+1]);
			}
			else if (map_node[i]->col == width){
				map_node[i]->neighbors.push_back(map_node[i-1]);
			}
			else{
				map_node[i]->neighbors.push_back(map_node[i+1]);
				map_node[i]->neighbors.push_back(map_node[i-1]);
			}

			//check lower left corner
			if (map_node[i]->row == 0 && map_node[i]->col == 0){
				map_node[i]->neighbors.push_back(map_node[i+width+1]);
			}
			//check lower right corner
			else if (map_node[i]->row == 0 && map_node[i]->col == width-1){
				map_node[i]->neighbors.push_back(map_node[i+width-1]);
			}
			//check upper left corner
			else if (map_node[i]->row == height-1 && map_node[i]->col == 0){
				map_node[i]->neighbors.push_back(map_node[i-width+1]);
			}
			//check upper right corner
			else if (map_node[i]->row == height-1 && map_node[i]->col == width-1){
				map_node[i]->neighbors.push_back(map_node[i-width-1]);
			}

			else if (map_node[i]->row == 0){
				map_node[i]->neighbors.push_back(map_node[i+width-1]);
				map_node[i]->neighbors.push_back(map_node[i+width+1]);
			}
			else if (map_node[i]->row == height-1){
				map_node[i]->neighbors.push_back(map_node[i-width+1]);
				map_node[i]->neighbors.push_back(map_node[i-width-1]);
			}
			else if (map_node[i]->col == 0){
				map_node[i]->neighbors.push_back(map_node[i+width+1]);
				map_node[i]->neighbors.push_back(map_node[i-width+1]);
			}
			else if (map_node[i]->col == width-1){
				map_node[i]->neighbors.push_back(map_node[i+width-1]);
				map_node[i]->neighbors.push_back(map_node[i-width-1]);
			}
			else{
				map_node[i]->neighbors.push_back(map_node[i+width-1]);
				map_node[i]->neighbors.push_back(map_node[i+width+1]);
				map_node[i]->neighbors.push_back(map_node[i-width+1]);
				map_node[i]->neighbors.push_back(map_node[i-width-1]);
			}





		}
	}

	bool FrontierDetermination(const shared_ptr<Node> current_node)
	{
		if (current_node->map_val <= 10 && current_node->map_val != -1){
			for (int i = 0; i < current_node->neighbors.size(); i++){
				if (current_node->neighbors[i]->map_val == -1){
					return true;
				}
				else{
					continue;
				}
			}
		}
		return false;
	}

	bool NodeValidity(const shared_ptr<Node> current_node)
	{
		for (int i = 0; i < current_node->neighbors.size(); i++){
			if (current_node->neighbors[i]->map_val <= 10 && current_node->neighbors[i]->map_val != -1){
				return true;
			}
			else{
				continue;
			}
		}
		return false;
	}

	void GetAngle(shared_ptr<Node> current_node){

		int iter = 0;
		double x_pos_tot = 0.0;
		double y_pos_tot = 0.0;
		double x_pos_avg = 0.0;
		double y_pos_avg = 0.0;

		for (int i = 0; i < current_node->neighbors.size(); i++){
			if (current_node->neighbors[i]->map_val == -1){
				x_pos_tot += current_node->neighbors[i]->pose.position.x;
				y_pos_tot += current_node->neighbors[i]->pose.position.y;
				iter++;
			}
		}
		x_pos_avg = x_pos_tot/iter;
		y_pos_avg = y_pos_tot/iter;
		double yaw = atan2(y_pos_avg - current_node->pose.position.y, x_pos_avg - current_node->pose.position.x);
		double roll;
		double pitch;

		double cy = cos(yaw*0.5);
		double sy = sin(yaw*0.5);
		double cp = cos(pitch*0.5);
		double sp = sin(pitch*0.5);
		double cr = cos(roll*0.5);
		double sr = sin(roll*0.5);

		current_node->pose.orientation.w = cr*cp*cy+sr*sp*sy;
		current_node->pose.orientation.x = sr*cp*cy-cr*sp*sy;
		current_node->pose.orientation.y = cr*sp*cy+sr*cp*sy;
		current_node->pose.orientation.z = cr*cp*sy-sr*sp*cy;

	}

	void ClearData()
	{
		map_raw.clear();
		map_queue.clear();
		map_node.clear();
		frontier_queue.clear();
		new_frontier.clear();
		pose_array.poses.clear();
		frontier_array.poses.clear();
		optimal_frontier_pts.poses.clear();
		frontier_list.clear();
	}

	void DoFrontier()
	{
		SetInitPose();
		MapConvert();
		GetNeighbors();
		PoseSnapInit();

		map_queue.push_back(current_node_map);
		current_node_map->map_open = true;

		while (!map_queue.empty()){

			current_node_map = map_queue.front();
			map_queue.pop_front();

			if (current_node_map->map_closed == true){
				continue;
			}
			else if (FrontierDetermination(current_node_map) == true){
				frontier_queue.push_back(current_node_map);
				current_node_map->frontier_open = true;

				while (!frontier_queue.empty()){

					current_node_frontier = frontier_queue.front();
					frontier_queue.pop_front();

					if (current_node_frontier->map_closed == true || current_node_frontier->frontier_closed == true){
						continue;
					}
					else if (FrontierDetermination(current_node_frontier) == true){
						GetAngle(current_node_frontier);
						new_frontier.push_back(current_node_frontier);

						for (int i = 0; i < current_node_frontier->neighbors.size(); i++){
							neighbor_node_frontier = current_node_frontier->neighbors[i];

							if (neighbor_node_frontier->frontier_open != true && neighbor_node_frontier->frontier_closed != true && neighbor_node_frontier->map_closed != true){
								frontier_queue.push_back(neighbor_node_frontier);
								neighbor_node_frontier->frontier_open = true;
							}
						}
					}
					current_node_frontier->frontier_closed = true;

				}
				frontier_list.push_back(new_frontier);
				for (int j = 0; j < new_frontier.size(); j++){
					new_frontier[j]->map_closed = true;
				}
				new_frontier.clear();
			}

			for (int k = 0; k < current_node_map->neighbors.size(); k++){

				neighbor_node_map = current_node_map->neighbors[k];

				if (neighbor_node_map->map_open != true && neighbor_node_map->map_closed != true && NodeValidity(neighbor_node_map) == true){
					map_queue.push_back(neighbor_node_map);
					neighbor_node_map->map_open = true;
				}
			}
			current_node_map->map_closed = true;
		}
		

	}

	static bool SortFunction( const vector< shared_ptr<Node>> v1, const vector< shared_ptr<Node>> v2 ) { 
	 	return v1.size() > v2.size(); 
	} 

	void SortFrontierArray(){
		std::sort(frontier_list.begin(), frontier_list.end(), &SortFunction);
	}

	static bool SortFunctionOptimal( const shared_ptr<Node> n1, const shared_ptr<Node> n2){
		return n1->optimal_heuristic < n2->optimal_heuristic;
	}

	void SortOptimalFrontier(vector< shared_ptr<Node> > &frontier_array){
		std::sort(frontier_array.begin(), frontier_array.end(), &SortFunctionOptimal);
	}

	void OptimalFrontierPtsOld(){

		for (int i = 0; i < frontier_list.size(); i++){
			
			if (frontier_list[i].size() > frontier_thresh){
				int median_val = int(frontier_list[i].size()/2);
				optimal_frontier_pts.poses.push_back(frontier_list[i][median_val]->pose);
			}
			
		}
	}

	void OptimalFrontierPts(){

		for (int i = 0; i < frontier_list.size(); i++){
			if (frontier_list[i].size() > frontier_thresh){

				double x_pos_tot = 0.0;
				double y_pos_tot = 0.0;
				double x_pos_avg = 0.0;
				double y_pos_avg = 0.0;
				int iter = 0;

				for (int j = 0; j < frontier_list[i].size(); j++){
					x_pos_tot += frontier_list[i][j]->pose.position.x;
					y_pos_tot += frontier_list[i][j]->pose.position.y;
					iter++;
				}

				x_pos_avg = x_pos_tot/iter;
				y_pos_avg = y_pos_tot/iter;

				for (int k = 0; k < frontier_list[i].size(); k++){
					double heuristic = sqrt(pow(x_pos_avg - frontier_list[i][k]->pose.position.x,2) + pow(y_pos_avg - frontier_list[i][k]->pose.position.y,2));
					frontier_list[i][k]->optimal_heuristic = heuristic;
				}

				SortOptimalFrontier(frontier_list[i]);
				optimal_frontier_pts.poses.push_back(frontier_list[i][0]->pose);


			}
		}


	}

	void NodeValuesTest(){
		for (int i = 0; i < map_node.size(); i++){
			frontier_array.poses.push_back(map_node[i]->pose);
		}
		cout << "size of frontier array: " << frontier_array.poses.size() << endl;
		frontier_array.header.frame_id = "map";
		frontier_array.header.stamp = ros::Time::now();
		while (ros::ok()){
			frontier_pub.publish(frontier_array);
		}
	}

	void FrontierPointTest(vector<shared_ptr<Node>> map_node){
		for (int i = 0; i < map_node.size(); i++){
			bool frontier_pt = FrontierDetermination(map_node[i]);
			if (frontier_pt == true){
				frontier_array.poses.push_back(map_node[i]->pose);

			}
		}
		cout << "size of frontier array: " << frontier_array.poses.size() << endl;
		frontier_array.header.frame_id = "map";
		frontier_array.header.stamp = ros::Time::now();
		while (ros::ok()){
			frontier_pub.publish(frontier_array);
		}
	}

	void FrontierArrayTest(vector<vector< shared_ptr<Node> > > frontier_list){

		// for (int i = 0; i < frontier_list.size(); i++){
		// 	for (int j = 0; j < frontier_list[i].size(); j++){
		// 		frontier_array.poses.push_back(frontier_list[i][j]->pose);
		// 	}
		// }

		for (int i = 0; i < frontier_list[0].size(); i++){
			frontier_array.poses.push_back(frontier_list[0][i]->pose);
		}

		// cout << "size of frontier array: " << frontier_array.poses.size() << endl;
		frontier_array.header.frame_id = "map";
		frontier_array.header.stamp = ros::Time::now();
		frontier_pub.publish(frontier_array);
		optimal_frontier_pt.header.frame_id = "map";
		optimal_frontier_pt.header.stamp = ros::Time::now();
		optimal_frontier_pt.pose = optimal_frontier_pts.poses[0];
		optimal_pub.publish(optimal_frontier_pt);


	}

	void OptimalFrontierPtsTest(){
		cout << "frontier_pts size: " << optimal_frontier_pts.poses.size() << endl;
		optimal_frontier_pts.header.frame_id = "map";
		optimal_frontier_pts.header.stamp = ros::Time::now();
		while (ros::ok()){
			frontier_pub.publish(optimal_frontier_pts);
		}

	}





};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "frontier_pts_node");
	ros::NodeHandle nh_;
	Frontier frontier;
	ros::ServiceServer map_service = nh_.advertiseService("/frontier_pts", &Frontier::MapCallback, &frontier);

	
	ros::spin();

	return 0;
}