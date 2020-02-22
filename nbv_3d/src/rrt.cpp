#include<nbv_3d/rrt.h>
#include<nbv_3d/process.h>
#include<nbv_3d/collision_check.h>
#include<chrono>
using namespace std::chrono;

//Note: have 2 cast ray in expand, but it won't be the bottleneck.

// 3D
// random sample: add z noise
// in expand, q_new.z() remove.


float dronesize_rrt = 0.3; //radius
float turtllebotsize_rrt = 0.2;
//bool rrt_2D = true;
bool rrt_2D = false;

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
	if (not rrt_2D){
		p.z() = p.z() + noise[noise_z_idx]; //for 2d case, don;t add any noise
	}
	return p;
}

void rrt::rrtBuild(point3d start, point3d goal, const OcTree& tree){
	bool found;
	point3d q_rand;
	setStart(start);
	setGoal(goal);
	this->res = tree.getResolution();
	// auto start_time = high_resolution_clock::now();
	while (true){
		q_rand = randomState();
		found = expand(q_rand, tree);
		// cout << "here" << endl;
		if (found == true){
			cout << "Path found!" << endl;
			break;
		}
		// auto stop_time = high_resolution_clock::now();
		// auto duration = duration_cast<seconds>(stop_time-start_time);
		// if (duration.count() > 1){
		// 	break;
		// }
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
	//cout << "rrt0" << endl;
	// print_point3d(q_rand);
	// print_point3d(q_near);
	q_new = adjustLength(q_rand, q_near);
	bool check_state;
	if (rrt_2D){
		//check_state = checkState(q_new, tree, this->res, turtllebotsize_rrt);
		check_state = true;
	}
	else{
		check_state = checkState(q_new, tree, this->res, dronesize_rrt);	
	}
	
	if (check_state == false){
		return false;
	}
	//cout << "rrt1" << endl;
	//avoid infinite loop
	vector<float> qv_near = point2vector(q_near);
	point3d q_last = parent[qv_near];
	if (q_new == q_last){
		return false;
	}

	//cout << "rrt2" << endl;

	// KeyRay ray;
	// bool connect_success;
	// tree.computeRayKeys(q_near, q_new, ray);
	// for (KeyRay::iterator itr=ray.begin(); itr != ray.end(); ++itr){
	// 	point3d checkPoint = tree.keyToCoord(*itr);
	// 	OcTreeNode* exist = tree.search(checkPoint);
	// 	// cout << "=========================================================================111" << endl;
	// 	bool checkPoint_state = checkState(checkPoint, tree, this->res, dronesize_rrt);
	// 	// cout << "=========================================================================222" << endl;
	// 	// cout << "=================check state======================= " << endl;
	// 	// cout << checkPoint_state << endl;
	// 	// cout << "=================check exist=======================" << endl;
	// 	// cout << (exist == NULL) << endl;
	// 	// if (exist != NULL){
	// 	// 	cout << "1" << endl;
	// 	// }

	// 	// if (checkPoint_state == true){
	// 	// 	cout << "2" << endl;
	// 	// }

		
	// 	//print_point3d(checkPoint);
	// 	if (exist == NULL or checkPoint_state == false){
	// 		return false;
	// 	}
	// }


	direction_cast = q_new - q_near;
	bool pass = tree.castRay(q_near, direction_cast, q_cast);
	distance_cast = q_near.distance(q_cast);
	// print_point3d(q_cast);
	// print_point3d(q_new);
	// print_point3d(q_near);
	// cout << distance_cast << endl;
	// cout << q_new.distance(q_near) << endl;
	//cout << pass << endl;
	//print_point3d(direction_cast);
	//print_point3d(q_cast);
	//cout << q_near.distance(q_new) << endl;
	//cout << distance_cast << endl;
	//print_point3d(q_new);
	//For 2D case:
	if (rrt_2D){
		q_new.z() = 0.175;
	} 
	if (distance_cast >= eps){
		nodes.push_back(q_new);
		vector<float> qv_new = point2vector(q_new);
		parent[qv_new] = q_near;
		//cout << eps << endl;
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

vector<point3d> rrt::shortCut(vector<point3d> path, const OcTree& tree){
	vector<point3d> new_path;
	double distance_cast;
	int size = path.size();
	int pos = 0;
	int count;
	bool pass;
	point3d direction_cast, p_cast;
	new_path.push_back(path[0]);
	// cout << "old path" << endl;
	// print_point3d_vector(path);
	while (pos < size){
		point3d p1 = path[pos];
		count = 2; //since 1 is not necessary to check
		if (pos+1 == size-1){
			new_path.push_back(path[pos+1]);
			break;
		}
		
		while (pos+count < size){
			point3d p2 = path[pos+count];
			direction_cast = p2 - p1;

			pass = tree.castRay(p1, direction_cast, p_cast);
			distance_cast = p1.distance(p_cast);
			if (distance_cast < p1.distance(p2)){
				new_path.push_back(path[pos+count-1]);
				break;
			}
			else{

				++count;
			}
			if (pos+count-1 == size-1){
				new_path.push_back(path[pos+count-1]);
				break;
			}
		}


		// while (pos+count < size){
		// 	point3d p2 = path[pos+count];
		// 	KeyRay ray;
		// 	tree.computeRayKeys(p1, p2, ray);
		// 	for (KeyRay::iterator itr=ray.begin()+1; itr != ray.end(); ++itr){
		// 		point3d checkPoint = tree.keyToCoord(*itr);
		// 		OcTreeNode* exist = tree.search(checkPoint);
		// 		bool checkPoint_state = checkState(checkPoint, tree, this->res, dronesize_rrt);
		// 		//print_point3d(checkPoint);
		// 		if (exist == NULL or checkPoint_state == false){
		// 			new_path.push_back(path[pos+count-1]);
		// 			break;
		// 		}
		// 	}
		// 	++count;
		// 	if (pos+count-1 == size-1){
		// 		new_path.push_back(path[pos+count-1]);
		// 		break;
		// 	}
		// }

		pos = pos+count-1;
	}
	return new_path;
}