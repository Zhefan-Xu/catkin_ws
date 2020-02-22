#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


using namespace message_filters; using std::cout; using std::endl;

ros::Publisher odom_pub;



void callback(const geometry_msgs::PoseConstPtr& pose, const geometry_msgs::TwistConstPtr& twist){
	int x = pose->position.x;
	int y = twist->linear.x;
	cout << x << endl;
	cout << y << endl;
	ROS_INFO("test");
}
	














int main(int argc, char** argv){
	ros::init(argc, argv, "convert odom");
	ros::NodeHandle nh;
	//ros::Subscriber sub = nh.subscribe("octomap_binary", 1000, callback);
	message_filters::Subscriber<geometry_msgs::Pose> pose_sub(nh, "/drone/gt_pose", 100);
	message_filters::Subscriber<geometry_msgs::Twist> twist_sub(nh, "/drone/gt_vel", 100);
	
	//typedef sync_policies::ApproximateTime<geometry_msgs::Pose, geometry_msgs::Twist> MySyncPolicy;
	//Synchronizer<MySyncPolicy> sync (MySyncPolicy(10), pose_sub, twist_sub);
	//path_pub = nh.advertise<nav_msgs::Path>("path", 1000);
	TimeSynchronizer<geometry_msgs::Pose, geometry_msgs::Twist> sync (pose_sub, twist_sub,100);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	ros::spin();

	return 0;
}