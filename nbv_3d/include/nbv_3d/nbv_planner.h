#include <ros/ros.h>
#include <nbv_3d/nbv.h>
#include <nbv_3d/rrt.h>
#include <nbv_3d/process.h>

// 3D
// p1.inflateOccupied(tree, 0.3)
// 2D
// p1.inflateOccupied(tree, 0.2)

vector<point3d> planning(OcTree tree, point3d origin, float& angle,float res, int sample_num, float rrt_eps){
	srand(time(NULL)); //set random seed
	point3d empty_point (0, 0, 0);
	point3d p (0, 0 ,0);
	NBVplanner p1 (res);
	p1.setOrigin(origin);
	
	p1.recordOccupied(tree);
	//p1.inflateOccupied(tree, 0.2);
	p1.inflateOccupied(tree, 0.3);
	p1.dataPro(tree);
	//tree.writeBinary("inflate_check.bt");
	vector<point3d> free = p1.getFree();
	//print_point3d_vector(free);
	//cout << "1" << endl;
	vector<point3d> occupied = p1.getOccupied();
	//cout << "2" << endl;
	KeySet frontier = p1.getFrontier();
	//cout << "3" << endl;
	sample sample_p;
	while (p == empty_point){
		sample_p = p1.findMaxPoint(sample_num, tree);
		p = sample_p.first;
		//cout << "here" << endl;
	}
	//cout << "4" << endl;
	angle = sample_p.second;
	rrt r (free, occupied, frontier, rrt_eps);
	point3d goal = sample_p.first;
	r.rrtBuild(origin, goal, tree);
	//cout << "5" << endl;
	vector<point3d> path = r.getPath();
	vector<point3d> new_path = r.shortCut(path, tree);
	cout << "Start: " << endl;
	print_point3d(origin); 
	cout << "Goal: " << endl;
	print_point3d(goal);
	//cout << "********************Path******************" << endl;
	//print_point3d_vector(new_path);
	return new_path;
}