
#include "ros/ros.h"
//#include "sensor_msgs/LaserScan.h"
#include <laser_geometry/laser_geometry.h>

laser_geometry::LaserProjection projector_;

class scan2pc{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Publisher pub;
	sensor_msgs::PointCloud2 pc;
	//sensor_msgs::PointCloud pc;
	laser_geometry::LaserProjection projector_;


public:
	scan2pc(){
		sub = nh.subscribe("modified_scan", 1000, &scan2pc::scanCallback, this);
		//pub = nh.advertise<sensor_msgs::PointCloud>("pc", 1000);
		pub = nh.advertise<sensor_msgs::PointCloud2>("pc", 1000);
	}

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
		projector_.projectLaser(*scan_in, pc);
		pub.publish(pc);
		ROS_INFO("convert successfully");
		//pub.publish(pc);
	}
};


int main(int argc, char** argv){
	ros::init(argc, argv, "scan2pc");
	scan2pc sc2pc;
	ros::spin();
	return 0;
}

/*
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class My_Filter {
     public:
        My_Filter();
        void scanCallback(const senor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};

My_Filter::My_Filter(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &My_Filter::scanCallback, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/pc", 100, false);
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_filter");

    My_Filter filter;

    ros::spin();

    return 0;
}
*/

