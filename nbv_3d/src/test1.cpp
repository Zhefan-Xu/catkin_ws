#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>


using std::cout; using std::endl; using std::vector;
using std::string;
using octomap::point3d;

class NBVplanner{
private:
	float res;


public:
	NBVplanner();
	point3d Sample(int num_s); //Return Point3d, 
						//input should be the number of sampling
	vector<point3d> DataPro();


};

//Default Constructor
NBVplanner::NBVplanner(){
	res = 0.1;
}


point3d NBVplanner::Sample(int num_s){
	for (int count = 0; count < num_s; ++count){
		//Sampling




	}
	point3d a (1.,2.,3.);
	cout << a.x() << a.y() << a.z() << endl;
	cout << "Success" << endl;
	return a;
} 


vector<point3d> DataPro(){
	vector<point3d> a;
	return a; 
}



int main(int argc, char** argv){
	//octomap::OcTree tree (0.1);
	NBVplanner p1;
	p1.Sample(5);
	return 0;
} 
