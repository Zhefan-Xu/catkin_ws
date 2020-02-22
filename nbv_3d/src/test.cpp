#include <ros/ros.h>
#include <nbv_3d/nbv_planner.h>

// larger rrt_eps is better 0.5 better than 0.2

float res = 0.1;

int main(int argc, char** argv){
	OcTree tree (0.1);
	tree.readBinary("check_rrt.bt");
	tree.setResolution(res);
	cout << "Resolution is: "<<tree.getResolution() << endl;
	//point3d origin (-3, 1, 0.35);
	//point3d origin (0.0, 0.94, 0.75);

	point3d origin (2, 15, 1);
	float angle;
	vector<point3d> path = planning(tree, origin, angle, res, 200, 0.5);
	tree.writeBinary("inflate_check.bt");
	cout << "Angle is: " << angle * 180/pi << " degree." << endl;
	cout << "Path: " << endl;
	print_point3d_vector(path);
	
	return 0;
}