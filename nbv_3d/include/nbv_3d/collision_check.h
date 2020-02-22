//#include <nbv_3d/rrt.h>
//#include <nbv_3d/process.h>

//3D
//k term in z direction

//bool check_2D = true;
bool check_2D = false;

bool checkState(point3d q, const OcTree& tree, float res,float rr){
	for (float i = q.x()-rr; i <= q.x()+rr; i += res){
		for (float j = q.y()-rr; j <= q.y()+rr; j += res){
			for (float k = q.z()-0.1; k <= q.z()+0.1; k += res){
				point3d temp (i, j, k);
				if (check_2D){
					temp.z() = q.z();
				}
				OcTreeNode* temp_ptr = tree.search(temp);
				if (temp_ptr == NULL){
					//continue;
					return false;
				}
				bool occupied = tree.isNodeOccupied(temp_ptr);
				if (occupied == true){
					//cout << "Occupied" << endl;
					return false;
				}
			}

		}
	}
	return true;
}

// return true means the state is safe


// bool checkState(point3d q, const OcTree& tree, float res,float rr){
// 	//cout << "check point is: " << endl;
// 	//print_point3d(q);
// 	//cout << "-----" << endl;
// 	for (float i = q.x()-rr; i <= q.x()+rr; i += res){
// 		for (float j = q.y()-rr; j <= q.y()+rr; j += res){
// 			point3d temp (i, j, q.z());
// 			//print_point3d(temp);
// 			OcTreeNode* temp_ptr = tree.search(temp);
// 			if (temp_ptr == NULL){
// 				//continue;
// 				//cout << "1" << endl;
// 				return false;
// 			}
// 			bool occupied = tree.isNodeOccupied(temp_ptr);
// 			if (occupied == true){
// 				//cout << "Occupied" << endl;
// 				//cout << "2" << endl;
// 				return false;
// 			}
// 		}
// 	}
// 	//cout << "here" << endl;
// 	return true;
// }
