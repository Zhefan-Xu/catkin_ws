#include <ros/ros.h>
#include <nbv_3d/pairAlign.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nbv_3d/nbv_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>



using namespace message_filters;
using namespace octomap;
using namespace sensor_msgs;
using namespace nav_msgs;

float res = 0.1;
OcTree tree(res);
octomap::Pointcloud pc_octo;
point3d goal (-1000, -1000, -1000);
point3d token  = goal;
ros::Publisher path_pub;
//for testing



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

/*
Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity ();
bool first_time = true;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
void callback(const OdometryConstPtr& odom, const PointCloud2ConstPtr& pc){
	float current_x = odom->pose.pose.position.x;
	float current_y = odom->pose.pose.position.y;
	float current_z = odom->pose.pose.position.z;
	float orientation_x = odom->pose.pose.orientation.x;
	float orientation_y = odom->pose.pose.orientation.y;
	float orientation_z = odom->pose.pose.orientation.z;
	float orientation_w = odom->pose.pose.orientation.w;
	float x, y, z, ox, oy, oz, ow;

	pcl::PCLPointCloud2 pcl_pc2;  //pc2 type
	pcl_conversions::toPCL(*pc, pcl_pc2); //ros_pc2 -> pcl_pc2
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_pc2, *cloud_tgt);
	Eigen::Matrix4f pair_transform;
	sensor_msgs::PointCloud2 output; // ros_pc2
	if (first_time){
		output = *pc;

		x = current_x;
		y = current_y;
		z = current_z;
		ox = orientation_x;
		oy = orientation_y;
		oz = orientation_z;
		ow = orientation_w;
		first_time = false;
	}
	else{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZ>);
		bool downsample = true;
		pairAlign(cloud_src, cloud_tgt, pair_transform, downsample); //DownSample == False by default
		global_transform *= pair_transform;
		pcl::transformPointCloud(*cloud_tgt, *cloud_aligned, global_transform);
		pcl::PCLPointCloud2 pc2_aligned;
		pcl::toPCLPointCloud2(*cloud_aligned, pc2_aligned); //pc -> pc2

		pcl_conversions::fromPCL(pc2_aligned, output); //pc2 -> ros_pc2
	}

	*cloud_src = *cloud_tgt;

	//point3d sensor_origin (current_x, current_y, current_z);
	point3d sensor_origin (0, 0, 0);
	//pose6d frame_origin (0, 0 ,0, 0, 0 ,0);
	point3d pos (current_x, current_y, current_z);
	//point3d pos (x, y, z);
	octomath::Quaternion rot (orientation_w, orientation_x, orientation_y, orientation_z);
	//octomath::Quaternion rot (ow, ox, oy, oz);
	pose6d frame_origin (pos, rot);
	//pose6d frame_origin (current_x, current_y, current_z, 0, 0, 0);
	octomap::pointCloud2ToOctomap(output, pc_octo);
	//point3d sensor_origin = point3d(current_x, current_y, current_z) - point3d(x, y, z);
	tree.insertPointCloud(pc_octo, sensor_origin, frame_origin);
	//tree.insertPointCloud(pc_octo, sensor_origin);
	ROS_INFO("Test");
	tree.writeBinary("check.bt");
}
*/

Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity ();
bool first_time = true;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
void callback(const OdometryConstPtr& odom, const PointCloud2ConstPtr& pc){
	float current_x = odom->pose.pose.position.x;
	float current_y = odom->pose.pose.position.y;
	float current_z = odom->pose.pose.position.z;
	float orientation_x = odom->pose.pose.orientation.x;
	float orientation_y = odom->pose.pose.orientation.y;
	float orientation_z = odom->pose.pose.orientation.z;
	float orientation_w = odom->pose.pose.orientation.w;
	float x, y, z, ox, oy, oz, ow;

	pcl::PCLPointCloud2 pcl_pc2;  //pc2 type
	pcl_conversions::toPCL(*pc, pcl_pc2); //ros_pc2 -> pcl_pc2
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_pc2, *cloud_tgt);
	Eigen::Matrix4f pair_transform;
	sensor_msgs::PointCloud2 output; // ros_pc2
	if (first_time){
		output = *pc;

		x = current_x;
		y = current_y;
		z = current_z;
		ox = orientation_x;
		oy = orientation_y;
		oz = orientation_z;
		ow = orientation_w;
		first_time = false;
	}
	else{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZ>);
		bool downsample = true;
		pairAlign(cloud_src, cloud_tgt, pair_transform, downsample); //DownSample == False by default
		global_transform *= pair_transform;
		pcl::transformPointCloud(*cloud_tgt, *cloud_aligned, global_transform);
		pcl::PCLPointCloud2 pc2_aligned;
		pcl::toPCLPointCloud2(*cloud_aligned, pc2_aligned); //pc -> pc2

		pcl_conversions::fromPCL(pc2_aligned, output); //pc2 -> ros_pc2
	}

	*cloud_src = *cloud_tgt;

	//map position for goal
	float map_goalx = goal.x();
	float map_goaly = goal.y();
	float map_goalz = goal.z();

	float eps_x = std::abs(map_goalx - current_x);
	float eps_y = std::abs(map_goaly - current_y);
	float eps_z = std::abs(map_goalz - current_z);
	bool reach = (eps_x < 0.1 and eps_y < 0.1 and eps_z < 0.1); 
	if (goal == token or reach){

		octomap::pointCloud2ToOctomap(output, pc_octo);
		point3d sensor_origin (0, 0, 0);
		point3d pos (current_x, current_y, current_z);
		octomath::Quaternion rot (orientation_w, orientation_x, orientation_y, orientation_z);

		pose6d frame_origin(pos, rot);  
		point3d origin = point3d(current_x, current_y, current_z);
		tree.insertPointCloud(pc_octo, sensor_origin, frame_origin);
		tree.writeBinary("check.bt");
		float angle;
		vector<point3d> path = planning(tree, origin, angle, res, 200, 0.3);
		//update origin 
		origin = path[path.size()-1];
		nav_msgs::Path path_msg = convertPathMsg(path, angle);
		print_point3d_vector(path);
		goal = path[path.size()-1];
		path_pub.publish(path_msg);
	}	
	
}


int main(int argc, char** argv){
	ros::init(argc, argv, "sync");
	//planner p;
	
	ros::NodeHandle nh;
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 100);
	message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "pc", 100);
	
	typedef sync_policies::ApproximateTime<Odometry, PointCloud2> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync (MySyncPolicy(10), odom_sub, pc_sub);
	path_pub = nh.advertise<nav_msgs::Path>("path", 1000);
	//TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2> sync (odom_sub, pc_sub,100);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	
	ros::spin();
}
