#ifndef NBV_H
#define NBV_H


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

#endif