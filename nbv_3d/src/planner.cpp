#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <unordered_set>
#include <time.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <octomap_ros/conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#define pi acos(-1)

using std::cout; using std::endl; using std::vector;
using std::set; using std::pair; using std::map;
using namespace octomap;
using namespace message_filters;
using namespace sensor_msgs;
using namespace nav_msgs;
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

class NBVplanner{
private:
	float res;
	float cam_r; //camera_radius
	float cam_angle; //camera_angle
	point3d origin;
	vector<point3d> free;
	vector<point3d> occupied;
	KeySet frontier;
public:
	NBVplanner(float res);
	void dataPro(const OcTree &tree); //get free and occupancy
	vector<point3d> getFree();
	vector<point3d> getOccupied();
	KeySet getFrontier();
	vector<point3d> getNeighbor(point3d p);
	point3d randomSample(); //helper method for findMaxPoint
	point3d randomDirection(); //helper method for findMaxPoint
	sample findMaxPoint(int sample_num,  const OcTree& tree); //find node with max gain
	int countFrontier(point3d p, const OcTree &tree, point3d direction);
	void setOrigin(point3d p);
};

NBVplanner::NBVplanner(float res){
	this->res = res;
	origin = point3d(0, 0 ,0);
	cam_r = 0.5;
	cam_angle = pi/2;
}


void NBVplanner::dataPro(const OcTree &tree){
	vector<point3d> neighbor; 
	for (OcTree::leaf_iterator itr = tree.begin_leafs(); 
		itr != tree.end_leafs(); ++itr){
		point3d p = itr.getCoordinate();
		OcTreeNode* nptr = tree.search(p);
		bool occ = tree.isNodeOccupied(nptr);
		if (occ == false){
			free.push_back(p);
			neighbor = getNeighbor(p);		
			for (vector<point3d>::iterator neighbor_itr = neighbor.begin();
					neighbor_itr != neighbor.end(); ++neighbor_itr){
				OcTreeNode* ptr = tree.search(*neighbor_itr);
				if (ptr == NULL){
					frontier.insert(tree.coordToKey(*neighbor_itr)); 	 
				}  
			}			  
		}
		else{
			occupied.push_back(p);
		}
	}
	cout << "Data Processing Finished!" << endl;
}

vector<point3d> NBVplanner::getFree(){return free;}
vector<point3d> NBVplanner::getOccupied(){return occupied;}
KeySet NBVplanner::getFrontier(){return frontier;}

vector<point3d> NBVplanner::getNeighbor(point3d p){
	float x = p.x();
	float y = p.y();
	float z = p.z();
	// +X -X +Y -Y +Z -Z
	point3d p_xp (x+res, y, z);
	point3d p_xn (x-res, y, z);
	point3d p_yp (x, y+res, z);
	point3d p_yn (x, y-res, z);
	point3d p_zp (x, y, z+res);
	point3d p_zn (x, y, z-res);
	vector<point3d> neighbor;
	neighbor.push_back(p_xp);
	neighbor.push_back(p_xn);
	neighbor.push_back(p_yp);
	neighbor.push_back(p_yn);
	neighbor.push_back(p_zp);
	neighbor.push_back(p_zn);
	return neighbor;
}

point3d NBVplanner::randomSample(){
	int rand_idx = rand() %  free.size();
	return free[rand_idx];
}

point3d NBVplanner::randomDirection(){
	vector<float> angled_array = {0, 45, 90, 135, 180, 225, 270, 315};
	int idx = rand() % angled_array.size(); //Random angle from 0~360
	float angled = angled_array[idx];
	float angle = angled*pi/180;
	point3d direction (cos(angle), sin(angle), 0);
	return direction;
}

sample NBVplanner::findMaxPoint(int sample_num, const OcTree& tree){
	point3d p, max_p (0, 0, 0);
	int num_frontier, max_frontier;
	float gain, max_gain=0;
	double distance;
	point3d direction, max_direction; 
	for (int i = 0; i < sample_num; ++i){
		p = randomSample();
		direction = randomDirection();
		num_frontier = countFrontier(p, tree, direction);
		distance = p.distance(origin);
		gain = num_frontier * exp(-0.1*distance);
		if (gain > max_gain){
			max_gain = gain;
			max_p = p;
			max_direction = direction;
			max_frontier = num_frontier;
		}
	}
	float angle = acos(direction.x());
	cout << "The maximum gain and frontier is: " << max_gain <<
	", " << max_frontier <<endl;
	//cout << "The direction is: " << endl;
	//print_point3d(max_direction);
	return std::make_pair(max_p, angle);
}

int NBVplanner::countFrontier(point3d p, const OcTree& tree, point3d direction){
	point3d f;
	double distance;
	//point3d direction (1, 0, 0); //Facing direction
	float angle;
	int count = 0;
	point3d direction_cast, endpoint;
	bool pass;
	for (KeySet::iterator frontier_itr = frontier.begin();
		frontier_itr != frontier.end(); ++frontier_itr){
		f = tree.keyToCoord(*frontier_itr);
		distance = p.distance(f);
		angle = direction.angleTo(f);
		// Cast ray to check if it could be seen
		direction_cast.x() = f.x() - p.x();
		direction_cast.y() = f.y() - p.y();
		direction_cast.z() = f.z() - p.z();
		pass = tree.castRay(p, direction_cast, endpoint);
		if (distance <= cam_r and angle <= cam_angle/2 and endpoint==f){
			++count;
		}
	}	
	return count;
}

void NBVplanner::setOrigin(point3d p){
	origin = p;
}


// RRT algorithm to navigate from one node to another node given the known space
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
};


rrt::rrt(vector<point3d> free, vector<point3d> occupied, KeySet frontier
			, double eps){
	this->free = free;
	this->occupied = occupied;
	this->frontier = frontier;
	this->eps = eps;
	token = point3d(-1000, -1000, -1000);
	res = 0.1;
	for (int i = 1; i < 100; ++i){
		noise.push_back(-res+i*res/50);
	}
}

point3d rrt::randomState(){
	int idx = rand() % free.size();
	float noise_x_idx = rand() % noise.size();
	float noise_y_idx = rand() % noise.size();
	float noise_z_idx = rand() % noise.size();
	point3d p = free[idx];
	p.x() = p.x() + noise[noise_x_idx];
	p.y() = p.y() + noise[noise_y_idx];
	p.z() = p.z() + noise[noise_z_idx];
	return p;
}

void rrt::rrtBuild(point3d start, point3d goal, const OcTree& tree){
	bool found;
	point3d q_rand;
	setStart(start);
	setGoal(goal);
	while (true){
		q_rand = randomState();
		found = expand(q_rand, tree);
		if (found == true){
			cout << "Path found!" << endl;
			break;
		}
	}
}

//argument q is the random sampling node
bool rrt::expand(point3d q_rand, const OcTree& tree){
	double distance, distance_cast;
	point3d q_new, q_cast, direction_cast;
	point3d q_near = nearestNeigbor(q_rand); //find the nn of q_rand
	/*
	distance = q_rand.distance(q_near);
	if (distance < eps){
		q_new = q_near;
	}
	else{
		q_new = adjustLength(q_rand, q_near);
	}
	*/
	q_new = adjustLength(q_rand, q_near);
	//avoid infinite loop
	vector<float> qv_near = point2vector(q_near);
	point3d q_last = parent[qv_near];
	if (q_new == q_last){
		return false;
	}
	direction_cast = q_new - q_near;
	bool pass = tree.castRay(q_near, direction_cast, q_cast);
	distance_cast = q_near.distance(q_cast);
	//cout << pass << endl;
	//print_point3d(direction_cast);
	//print_point3d(q_cast);
	//cout << q_near.distance(q_new) << endl;
	//cout << distance_cast << endl;
	if (distance_cast >= eps){
		nodes.push_back(q_new);
		vector<float> qv_new = point2vector(q_new);
		parent[qv_new] = q_near;
		//cout << q_new.distance(goal) << endl;
		if (q_new.distance(goal) <= eps){
			nodes.push_back(goal);
			vector<float> goalv = point2vector(goal); 
			parent[goalv] = q_new;
			return true;
		}
	}
	return false;
}

point3d rrt::nearestNeigbor(point3d q){
	double min_distance = 1000;
	point3d q_near;
	double distance;
	for (vector<point3d>::iterator node_itr = nodes.begin();
			node_itr != nodes.end(); ++node_itr){
		distance = q.distance(*node_itr);
		if (distance < min_distance){
			min_distance = distance;
			q_near = *node_itr;
		}
	}
	return q_near;
}

point3d rrt::adjustLength(point3d q_rand, point3d q_near){
	//return the adjusted q_new
	point3d q_new;
	point3d direction = q_rand - q_near; //get the vector direction
	direction /= direction.norm(); //normalize;
	direction *= eps;
	q_new = direction + q_near;
	return q_new;
}

void rrt::setStart(point3d p){
	start = p;
	nodes.push_back(start);
	//nodes.push_back(point3d (1,2,3));
	//nodes.push_back(point3d (4,4,4));
	vector<float> startv = point2vector(start);
	parent[startv] = token;
}

void rrt::setGoal(point3d p){goal = p;}

vector<point3d> rrt::getPath(){
	point3d p = goal;
	vector<point3d> path;
	path.insert(path.begin(), goal);
	int count = 0;
	while (true){
		vector<float> pv = point2vector(p);
		point3d p_parent = parent[pv];
		if (p_parent == token){
			break;
		}
		path.insert(path.begin(), p_parent);
		p = p_parent;
		++count;
		/*
		if (count > 100){
			cout << "infinite loop" << endl;
			print_point3d(p_parent);
		}
		*/
	}
	return path;
}

map<vector<float>, point3d> rrt::getParent(){return parent;}


//planning class
class planning{
private:
	ros::NodeHandle nh;
	
public:
	planning(){
		message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "pc", 1);
		message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 1);
		TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync (pc_sub, odom_sub, 10);
		sync.registerCallback(boost::bind(&planning::callback, this, _1, _2));
		cout << "here" << endl;
	}


	void callback(const PointCloud2ConstPtr& pc, const OdometryConstPtr& odom){
		float x = odom->pose.pose.position.x;
		//ROS_INFO("x: %f", x);
		ROS_INFO("test");
	}
	/*
	void pcCallback(const sensor_msgs::PointCloud2 pc_){
		octomap::pointCloud2ToOctomap(pc_, pc_octo);
	}

	void odomCallback(const nav_msgs::Odometry odom_){
		x = odom_.pose.pose.position.x;
		y = odom_.pose.pose.position.y;
		z = odom_.pose.pose.position.z;
		check = true;
	}

	void update(OcTree &tree){
		//origin
		//tree

		if (check == false){
			ros::spin();
		}
		origin.x() = x;
		origin.y() = y;
		origin.z() = z; 
		tree.insertPointCloud(pc_octo, origin);
	}

	void test(){
		print_point3d(origin);
		OcTree tree (0.1);
		update(tree);
		while (true){
			origin.x() = x;
			origin.y() = y;
			origin.z() = z; 
			tree.insertPointCloud(pc_octo, origin);
			tree.writeBinary("tree2.bt");
		}	

		
		

		//cout << x << endl;
		//cout << m << endl;
		//print_point3d(origin);
		
	}
*/
};



int main(int argc, char** argv){
	srand(time(NULL)); //set random seed
	OcTree tree ("test_tree1.bt");
	NBVplanner p1 (0.1);
	p1.dataPro(tree);
	vector<point3d> free = p1.getFree();
	vector<point3d> occupied = p1.getOccupied();
	KeySet frontier = p1.getFrontier();
	cout << "The size of free is: " << free.size() << endl; 
	cout << "The size of occupied is: " << occupied.size() << endl;
	cout << "The size of frontier is: " << frontier.size() << endl;
	//print_point3d_vector(free);
	/*
	point3d sample = p1.randomSample();
	print_point3d(sample);
	
	point3d direction = p1.randomDirection();
	cout << "Random Direction is: " << endl;
	print_point3d(direction);
	*/
	sample sample_p = p1.findMaxPoint(200, tree);
	cout << "Position is: " << endl;
	print_point3d(sample_p.first);
	cout << "Direction angle is: " << endl;
	cout << sample_p.second*180/pi << " degree" << endl;


	/*
	OcTreeNode* p_test = tree.search(0.61, 1.22, 0.05);
	bool occ = tree.isNodeOccupied(p_test);
	cout << occ << endl;
	*/


	/*
	************BELOW IS TEST FOR RRT *************************
	*/
	cout << "************BELOW IS TEST FOR RRT***********" << endl;
	rrt r (free, occupied, frontier, 0.3);
	point3d q_rand = r.randomState();
	print_point3d(q_rand);



	point3d start (0, 0, 0.05);
	point3d goal = sample_p.first;
	/*
	r.setStart(start);
	r.setGoal(goal);
	point3d q_near = r.nearestNeigbor(point3d(3, 3, 4));
	cout << "Nearest Neighbor: " << endl;
	print_point3d(q_near);

	cout << "adjust length: " << endl;
	point3d q_new = r.adjustLength(point3d (5, 4, 3), point3d (0, 1 ,2));
	print_point3d(q_new);
	*/
	r.rrtBuild(start,goal, tree);
	map<vector<float>, point3d> parent = r.getParent();
	cout << "size of map is: " << parent.size() << endl;
	//print_point3d(parent[point2vector(goal)]);
	//point3d p_last = parent[point2vector(goal)];
	//print_point3d(parent[point2vector(p_last)]);
	vector<point3d> path = r.getPath();
	print_point3d_vector(path);


	cout << "************BELOW IS TEST FOR PLANNNER*********" << endl;
	ros::init(argc, argv, "planning");
	planning test_p;
	ros::spin();


	return 0;
}


/*
			if (count == 0){
			print_point3d(p);
			cout << res << endl;
			cout << neighbor[0] << endl;
			cout << neighbor[1] << endl;
			cout << neighbor[2] << endl;
			cout << neighbor[3] << endl;
			cout << neighbor[4] << endl;
			cout << neighbor[5] << endl;
			++count;
			}


	point3d pp (0, 1, 2);
	vector<float> pv = point2vector(pp);
	cout << pv[0] << pv[1] << pv[2] << endl;

	point3d ppp = vector2point(pv);
	print_point3d(ppp);
*/