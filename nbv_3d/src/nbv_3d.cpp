#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>


void test(const std::string& filename){
	octomap::OcTree tree (filename);
	//bool s1 = tree.writeBinary("simple_tree2.bt");
	//bool success = tree.readBinary(filename);
	//std::cout << s1 << std::endl;
	int count = 0;
	//octomap::OcTree::leaf_bbx_iterator itr;
	octomap::OcTree::leaf_iterator itr;
	octomap::OcTree::iterator tree_itr;
	octomap::point3d min_point (0.02, 0.0, 0.05);
	octomap::point3d max_point (0.1, 0.1, 0.1);
	
	//for (itr = tree.begin_leafs_bbx(min_point, max_point); itr != tree.end_leafs_bbx(); ++itr){
	for (itr = tree.begin_leafs(); itr != tree.end_leafs(); ++itr){
	//for (tree_itr = tree.begin(); tree_itr != tree.end(); ++tree_itr){
		if (count >= 0 ){

			//std::cout << "Node Center: " << tree_itr.getCoordinate() << std::endl;
			octomap::point3d point = itr.getCoordinate();
			octomap::OcTreeNode* nptr = tree.search(point);
			if (nptr == NULL){
				std::cout << "NULL" << std::endl;
			}
			bool occupied = tree.isNodeOccupied(nptr);
			std::cout << "Occupancy: " << occupied << std::endl;
			//std::cout << "Node Size: " << itr.getSize() << std::endl;
			//std::cout << "Node value: " << itr->getValue() << std::endl;
			//octomap::OcTreeKey key = itr.getKey();

			//std::cout << "Node key: " << key[0]<< ", " << key[1] << ", " << key[2] << std::endl;
	}
	
	
		++count;
}
	
	std::cout << "Total Number of Iteration: " << count << std:: endl;
	

}



int main(int argc, char** argv){
	test("test_tree.bt");
	return 0;
}
