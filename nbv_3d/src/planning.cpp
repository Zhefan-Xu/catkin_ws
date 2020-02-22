#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nbv_3d/nbv_planner.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <iterator>
#include <string>

using namespace message_filters;

//3D
// remove set z = 0
// change origin

//bool planning_2D = true;
bool planning_2D = false;



ros::Publisher goal_pub;
int idx = 1;
ros::Publisher path_pub;
//ros::Publisher path_sofar_pub;
nav_msgs::Path path_msg;
//nav_msgs::Path path_sofar_msg;
float res = 0.1;
OcTree tree(res);
point3d goal (-1000, -1000, -1000);
vector<point3d> path;
//vector<point3d> path_sofar;
point3d token  = goal;
std::ofstream output_file("./path.txt");
float total_length = 0;

nav_msgs::Path convertPathMsg(vector<point3d> path, float angle){
	nav_msgs::Path path_msg;
	vector<geometry_msgs::PoseStamped> msg;
	for (int i = 0; i < path.size(); ++i){
		geometry_msgs::Point p;
		geometry_msgs::PoseStamped ps;
		geometry_msgs::Quaternion q;
		p.x = path[i].x();
		p.y = path[i].y();
		p.z = path[i].z();
		ps.pose.position = p;
		msg.push_back(ps);
	}
	path_msg.poses = msg;
	return path_msg;
}

void writePath(vector<point3d> path){
	float path_length = 0;

	vector<std::string> seperate;
	seperate.push_back("========================================================");
	std::ostream_iterator<std::string> seperate_iterator(output_file, "\n");
	std::copy(seperate.begin(), seperate.end(), seperate_iterator);

	for (int i=0; i<path.size(); ++i){
		vector<std::string> seperate_p;
		seperate_p.push_back("------------------------------------------");
		std::ostream_iterator<std::string> seperate_p_iterator(output_file, "\n");
		std::copy(seperate_p.begin(), seperate_p.end(), seperate_p_iterator);
		float x = path[i].x();
		float y = path[i].y();
		float z = path[i].z();
		vector<float> p;
		p.push_back(x);
		p.push_back(y);
		p.push_back(z);
		std::ostream_iterator<float> output_iterator(output_file, " ");
		std::copy(p.begin(), p.end(), output_iterator);
		vector<std::string> seperate_empty;
		seperate_empty.push_back(" ");
		std::ostream_iterator<std::string> seperate_empty_iterator(output_file, "\n");
		std::copy(seperate_empty.begin(), seperate_empty.end(), seperate_empty_iterator);
		if (i != 0){
			path_length += path[i].distance(path[i-1]);
		}
	}
	total_length += path_length;
	vector<float> path_length_v;
	path_length_v.push_back(path_length);
	path_length_v.push_back(total_length);
	std::ostream_iterator<float> path_length_iterator(output_file, "\n");
	std::copy(path_length_v.begin(), path_length_v.end(), path_length_iterator);
	cout << "Path lenght is: " << path_length << endl;
	cout << "Total length is: " << total_length << endl;
}

void callback(const nav_msgs::OdometryConstPtr& odom, const octomap_msgs::Octomap::ConstPtr& bmap){
	OcTree* tree_ptr= new OcTree(res);
	AbstractOcTree* abtree = octomap_msgs::binaryMsgToMap(*bmap);
	tree_ptr = dynamic_cast<OcTree*>(abtree);
	
	//(*tree_ptr).writeBinary("map.bt");
	//cout << (*tree_ptr).getResolution() << endl;

	float current_x = odom->pose.pose.position.x;
	float current_y = odom->pose.pose.position.y;
	float current_z = odom->pose.pose.position.z;
	float orientation_x = odom->pose.pose.orientation.x;
	float orientation_y = odom->pose.pose.orientation.y;
	float orientation_z = odom->pose.pose.orientation.z;
	float orientation_w = odom->pose.pose.orientation.w;


	//map position for goal
	float map_goalx = goal.x();
	float map_goaly = goal.y();
	float map_goalz = goal.z();

	float eps_x = std::abs(map_goalx - current_x);
	float eps_y = std::abs(map_goaly - current_y);
	float eps_z = std::abs(map_goalz - current_z);
	if (planning_2D){
		eps_z = 0; //only for 2d case	
	}
	bool reach = (eps_x < 0.1 and eps_y < 0.1 and eps_z < 0.1); 
	if (goal == token or reach){

		//octomap::pointCloud2ToOctomap(output, pc_octo);
		//point3d sensor_origin (0, 0, 0);
		//point3d pos (current_x, current_y, current_z);
		//octomath::Quaternion rot (orientation_w, orientation_x, orientation_y, orientation_z);

		//pose6d frame_origin(pos, rot);

		// point3d origin (current_x, current_y, 0.175);
		point3d origin (current_x, current_y, current_z);
		if (planning_2D){
			origin.z() = 0.175;
			//print_point3d(origin);
		}
		//tree.insertPointCloud(pc_octo, sensor_origin, frame_origin);
		//(*tree_ptr).writeBinary("check_rrt.bt");
		float angle;
		if (planning_2D){
			path = planning(*tree_ptr, origin, angle, res, 400, 0.2);	
		}
		else{
			path = planning(*tree_ptr, origin, angle, res, 400, 0.5);	
		}
		//path_sofar.insert(path_sofar.end(), path.begin(), path.end());
		writePath(path);
		//update origin 
		origin = path[path.size()-1];
		path_msg = convertPathMsg(path, angle);
		//path_sofar_msg = convertPathMsg(path_sofar, angle);
		path_msg.header = odom->header;
		cout<<"Path is:" << endl;
		print_point3d_vector(path);
		goal = path[path.size()-1];
		idx = 1;
	}
	path_pub.publish(path_msg);
	//path_sofar_pub.publish(path_sofar_msg);
	if (not planning_2D){	
		geometry_msgs::Point next_position;
		next_position.x = path[idx].x();
		next_position.y = path[idx].y();
		next_position.z = path[idx].z();
		float next_eps_x = std::abs(next_position.x - current_x);
		float next_eps_y = std::abs(next_position.y - current_y);
		float next_eps_z = std::abs(next_position.z - current_z);
		bool reach_next = (next_eps_x < 0.1 and next_eps_y < 0.1 and next_eps_z < 0.1);
		if (not reach_next){
			goal_pub.publish(next_position);
		}
		else{
			idx += 1;
		}
	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "planning");
	ros::NodeHandle nh;
	//ros::Subscriber sub = nh.subscribe("octomap_binary", 1000, callback);
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 100);
	message_filters::Subscriber<octomap_msgs::Octomap> map_sub(nh, "octomap_binary", 100);
	
	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, octomap_msgs::Octomap> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync (MySyncPolicy(100), odom_sub, map_sub);
	path_pub = nh.advertise<nav_msgs::Path>("path", 1);
	//path_sofar_pub = nh.advertise<nav_msgs::Path>("path_sofar", 1);
	goal_pub = nh.advertise<geometry_msgs::Point>("/get_goal", 1);
	//TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2> sync (odom_sub, pc_sub,100);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	ros::spin();

	return 0;
}