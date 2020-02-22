#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <unordered_set>
#include <time.h>
#include <math.h>
#define pi acos(-1)

using std::cout; using std::endl; using std::vector;
using std::set; using std::pair; using std::map;
using namespace octomap;
typedef pair<point3d, float> sample;
#ifndef RRT_H
#define RRT_H

class rrt{
private:
	float res;
	double eps; //maximum increament length
	// free, occupied, frontier
	point3d start;
	point3d goal;
	point3d token; //special use
	vector<point3d> free;
	vector<point3d> occupied;
	KeySet frontier;
	//unordered_map<point3d, point3d> parent; 
	// using a map and a vector to record the tree node and its relation
	map<vector<float>, point3d> parent;
	vector<point3d> nodes;
	vector<float> noise;

public:
	rrt(vector<point3d> free, vector<point3d> occupied, KeySet frontier, double eps); // constructor
	point3d randomState(); // return random node q
	void rrtBuild(point3d start, point3d goal, const OcTree& tree);
	bool expand(point3d q_rand,const OcTree& tree); //expand tree
	//belows are helper function for expand
	point3d nearestNeigbor(point3d q);
	point3d adjustLength(point3d q_rand, point3d q_near);
	void setStart(point3d p);
	void setGoal(point3d p);
	vector<point3d> getPath();
	map<vector<float>, point3d> getParent();
	vector<point3d> shortCut(vector<point3d> path, const OcTree& tree);
};

#endif