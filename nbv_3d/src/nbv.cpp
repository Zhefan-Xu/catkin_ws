#include <nbv_3d/nbv.h>
#include <nbv_3d/process.h>
#include <nbv_3d/collision_check.h>

// Note:
// Trial1: I am not using ray casting for counting frontier, too expenisve
// Trial2: Check other condition first, works well

// 3D
// camera angle -> pi/2
// robot size 0.5 * 0.5 * 0.1 
// Count Frontier condition: remove
// camera radius 3


//bool nbv_2D = true;
bool nbv_2D = false;
//bool tunnel = false;
bool tunnel = true;
float dronesize_nbv = 0.3; //radius
float turtlebotsize_nbv = 0.2; //radius


NBVplanner::NBVplanner(float res){
	this->res = res;
	origin = point3d(0, 0 ,0);
	cam_r = 0.3;
	//cam_r = 1;
	//cam_angle = pi/2;
	cam_angle = pi*2;
}

void NBVplanner::inflateOccupied(OcTree& tree, float radius){
	for (int i=0; i<occupied.size(); ++i){
		point3d p = occupied[i];
		// Infalte map
		for (float i = p.x()-radius; i <= p.x()+radius; i+=res){
			for (float j = p.y()-radius; j <= p.y()+radius; j+=res){
				for (float k = p.z()-0; k <= p.z()+0; k+=res){
					point3d temp (i, j, k);
					// if (nbv_2D){
					// 	temp.z() = 0.175;
					// }
					//OcTreeNode temp_ptr = tree.search(temp);
					tree.updateNode(temp, true);					
					}
				}
			}
		}
	cout << "Inflation Finished!" << endl;
}


void NBVplanner::recordOccupied(const OcTree &tree){
	for (OcTree::leaf_iterator itr = tree.begin_leafs(); 
		itr != tree.end_leafs(); ++itr){
		point3d p = itr.getCoordinate();
		OcTreeNode* nptr = tree.search(p);
		bool occ = tree.isNodeOccupied(nptr);
		if (occ == true){
			occupied.push_back(p);		  
		}
	}
}


void NBVplanner::dataPro(const OcTree &tree){
	occupied.clear();
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
	// cout << free.size() << endl;
	// cout << occupied.size() << endl;
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

point3d NBVplanner::randomSample(const OcTree& tree){
	point3d p;
	bool check_state = false;
	while (check_state == false){
		int rand_idx = rand() %  free.size();
		p = free[rand_idx];
		// cout << "here" << endl;
		if (nbv_2D){
			check_state = checkState(p, tree, this->res, turtlebotsize_nbv);
		}
		else{
			check_state = checkState(p, tree, this->res, dronesize_nbv);
		}
	}
	
	return p;
}

point3d NBVplanner::randomDirection(){
	// vector<float> angled_array = {0, 45, 90, 135, 180, 225, 270, 315};
	// int idx = rand() % angled_array.size(); //Random angle from 0~360
	// float angled = angled_array[idx];
	float angled = rand() % 361;
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
		p = randomSample(tree);

		direction = randomDirection();
		num_frontier = countFrontier(p, tree, direction);
		distance = p.distance(origin);
		gain = num_frontier * exp(-0.3*distance);
		if (tunnel){
			if (p.y() <= 1 or p.z() <= 1){
				gain -= 100000;
			}
		}
		// if (p.z() >= 9 or p.y() <= 1 or p.y() <= 1){
		// 	gain -= 100000;
		// }
		// cout << i << endl;
		if (gain > max_gain){
			max_gain = gain;
			max_p = p;
			max_direction = direction;
			max_frontier = num_frontier;
		}
	}
	float angle = acos(direction.x());
	if (max_p == point3d (0, 0, 0)){}
	else{
		cout << "The maximum gain and frontier is: " << max_gain <<
		", " << max_frontier <<endl;
		print_point3d(max_p);
	}
	//cout << "The direction is: " << endl;
	//print_point3d(max_direction);
	return std::make_pair(max_p, angle);
}

int NBVplanner::countFrontier(point3d p, const OcTree& tree, point3d direction){
	point3d f;
	double distance, cast_distance;
	//point3d direction (1, 0, 0); //Facing direction
	float angle;
	float ref = 0.175;
	int count = 0;
	point3d direction_cast, endpoint;
	bool pass;
	for (KeySet::iterator frontier_itr = frontier.begin();
		frontier_itr != frontier.end(); ++frontier_itr){
		f = tree.keyToCoord(*frontier_itr);
		//cout << f.z() << endl;
		//bool inBound = (f.z() == p.z());
		// nbv_2D
		if (nbv_2D){
			bool inBound = (f.z() == ref);
			if (inBound == false){
				continue;
			}
		} 
		distance = p.distance(f);
		angle = direction.angleTo(f);
		// Cast ray to check if it could be seen
		// direction_cast.x() = f.x() - p.x();
		// direction_cast.y() = f.y() - p.y();
		// direction_cast.z() = f.z() - p.z();
		// pass = tree.castRay(p, direction_cast, endpoint);
		// //cout << f.z() << endl;
		
		// cast_distance = p.distance(endpoint);
		// bool cast = std::abs(cast_distance - distance) < res;

		bool cast = true;
		//cout << cast << endl;
//and endpoint==f

		if (distance <= cam_r and angle <= cam_angle/2){
			// Cast ray to check if it could be seen
			direction_cast.x() = f.x() - p.x();
			direction_cast.y() = f.y() - p.y();
			direction_cast.z() = f.z() - p.z();
			pass = tree.castRay(p, direction_cast, endpoint);
			//cout << f.z() << endl;
			
			cast_distance = p.distance(endpoint);
			bool cast = std::abs(cast_distance - distance) < res;
			if (cast == true){
				++count;
			}

		}
	}	
	return count;
}

void NBVplanner::setOrigin(point3d p){
	origin = p;
}


