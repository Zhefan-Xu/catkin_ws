#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
/*

class pcConvertor{
private:
	ros::NodeHandle nh;
	octomap::Pointcloud pc_octo; //point cloud data in octomap


public:
	pcConvertor(){
		ros::Subscriber sub = nh.subscribe("pc", 100, &pcConvertor::conversion_callback, this);
		ros::Publisher pub = nh.advertise<octomap::>
	}

	void conversion_callback(const sensor_msgs::PointCloud2 pc){

	}

}

*/

void conversion_callback(const sensor_msgs::PointCloud2 pc){
	octomap::Pointcloud pc_octo;
	octomap::pointCloud2ToOctomap(pc, pc_octo);
	//octomap::pointCloud2ToOctomap convertor;
	//convertor.PointCloud2ToOctomap(pc, pc_octo);
	octomap::OcTree tree (0.3);
	octomap::point3d origin (0., 0., 0.);
	//octomap::point3d origin (-2, -0.5, 0);
	tree.insertPointCloud(pc_octo, origin);
	tree.writeBinary("test_tree4.bt");
	octomap::point3d query (1., 1., 1.);
	//octomap::OcTreeNode* result = tree.search(query);
	//bool a = result != NULL;

	//std::cout << a << std::endl;
	ROS_INFO("test");
}






int main(int argc, char** argv){
	ros::init(argc, argv, "pc_convertor");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("pc", 100, conversion_callback);
	ros::spin();
	return 0;
}