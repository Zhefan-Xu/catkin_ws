#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <time.h>
#include <math.h>
#define pi acos(-1)
using std::cout; using std::endl; using std::vector;
using std::set; using std::pair; using std::map;
using namespace octomap;
typedef pair<point3d, float> sample;

void print_point3d(point3d p){
	cout << "The point: " << "(" << p.x() << ", " <<
	p.y() << ", " << p.z() << ")" << endl;	
}

void print_point3d_vector(vector<point3d> p){
	int count = 0;

	for (vector<point3d>::iterator itr = p.begin(); 
			itr != p.end(); ++itr){
		print_point3d(*itr);

		/*
			cout << "Point Num: " << count+1 << endl; 
			cout 
			<< "x: " << itr->x() << endl 
			<< "y: " << itr->y() << endl 
			<< "z: " << itr->z() << endl;
		*/

	}
}


vector<float> point2vector(point3d p){
	vector<float> v;
	v.push_back(p.x());
	v.push_back(p.y());
	v.push_back(p.z());
	return v;
}

point3d vector2point(vector<float> v){
	point3d p (v[0], v[1], v[2]);
	return p;
}
