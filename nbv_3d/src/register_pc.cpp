

ros::Publisher pub;



Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity ();
bool first_time = true;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
void pointcloudcb(const sensor_msgs::PointCloud2::ConstPtr& pc){
	pcl::PCLPointCloud2 pcl_pc2;  //pc2 type
	pcl_conversions::toPCL(*pc, pcl_pc2); //ros_pc2 -> pcl_pc2
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_pc2, *cloud_tgt);
	Eigen::Matrix4f pair_transform;
	if (first_time){
		pub.publish(pc);
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
		sensor_msgs::PointCloud2 output; // ros_pc2
		pcl_conversions::fromPCL(pc2_aligned, output); //pc2 -> ros_pc2
		pub.publish(output);
	}

	*cloud_src = *cloud_tgt;
	ROS_INFO("test");
}




int main(int argc, char** argv){
	ros::init(argc, argv, "register_pc");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("pc", 1000, pointcloudcb);
	pub = nh.advertise<sensor_msgs::PointCloud2>("aligned_pc", 1000);
	ros::spin();
	return 0;
}